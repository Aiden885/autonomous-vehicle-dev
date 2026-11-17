#include "apa_motion_prediction.h"
#include <algorithm>
#include <cmath>
#include <limits>

namespace pangu::modules {
ApaMotionPose AdvanceApaMotionPose(ApaMotionPose origin,
    double longitudinal, double lateral, double angle) {
  const double a=std::fabs(angle)>1.e-9 ? std::sin(angle)/angle : 1.0;
  const double b=std::fabs(angle)>1.e-9 ? (1.0-std::cos(angle))/angle : 0.0;
  const double x=a*longitudinal-b*lateral, y=b*longitudinal+a*lateral;
  return {origin.x+std::cos(origin.yaw)*x-std::sin(origin.yaw)*y,
          origin.y+std::sin(origin.yaw)*x+std::cos(origin.yaw)*y,origin.yaw+angle};
}

ApaTrackingMotionPredictor::ApaTrackingMotionPredictor(ApaMotionPose origin,
    const ApaMotionPathView& path, const ApaMotionPredictionPolicy& policy,
    const ApaMotionPrediction& motion, double signed_speed) : policy_(policy) {
  if (!policy.braking.Valid || !std::isfinite(policy.wheelbase) || policy.wheelbase<=0 ||
      !std::isfinite(policy.braking.Distance) || policy.braking.Distance<0 ||
      !std::isfinite(policy.braking.StopTime) || policy.braking.StopTime<0 ||
      !std::isfinite(policy.braking.ReactionTime) || policy.braking.ReactionTime<0 ||
      !std::isfinite(policy.braking.Jerk) || policy.braking.Jerk<=0 ||
      !std::isfinite(policy.braking.Deceleration) || policy.braking.Deceleration<=0 ||
      !std::isfinite(policy.minimum_steer) || !std::isfinite(policy.maximum_steer) ||
      !std::isfinite(policy.steer_rate_per_sec) || policy.steer_rate_per_sec<0 ||
      !std::isfinite(policy.steering_response_time) || policy.steering_response_time<0 ||
      (motion.response_reference_valid && (!std::isfinite(motion.reference_curvature) ||
       !std::isfinite(motion.steering_angle))) ||
      !std::isfinite(policy.speed_response_time) || policy.speed_response_time<0 ||
      policy.minimum_steer>0 || policy.maximum_steer<0 ||
      policy.minimum_steer<=-1.57079632679 || policy.maximum_steer>=1.57079632679 ||
      !std::isfinite(origin.x) || !std::isfinite(origin.y) || !std::isfinite(origin.yaw) ||
      !std::isfinite(signed_speed) || !std::isfinite(motion.commanded_curvature) ||
      !std::isfinite(path.progress) || path.progress<0 ||
      path.count<2 || !path.x || !path.y || !path.yaw ||
      !path.odometer || (path.direction!=1 && path.direction!=-1)) return;
  double length=0;
  for (int i=0;i+1<path.count;++i) {
    const double dx=double(path.x[i+1])-path.x[i], dy=double(path.y[i+1])-path.y[i];
    const double odo=path.odometer[i];
    const double yaw=double(path.yaw[i+1])-path.yaw[i];
    if (!std::isfinite(dx) || !std::isfinite(dy) || !std::isfinite(yaw) ||
        !std::isfinite(odo) || path.direction*odo<0) return;
    if (std::hypot(dx,dy)<=1.e-7 || std::fabs(odo)<=1.e-7) continue;
    length+=std::hypot(dx,dy);
    sections_.push_back({length,std::atan2(std::sin(yaw),std::cos(yaw))/odo});
  }
  if (sections_.empty()) return;
  progress_=path.progress;
  correction_=motion.commanded_curvature-(motion.response_reference_valid ?
      motion.reference_curvature : reference_curvature(progress_));
  initial_steering_=std::atan(policy.wheelbase*motion.commanded_curvature);
  if (initial_steering_<policy.minimum_steer-1.e-5 ||
      initial_steering_>policy.maximum_steer+1.e-5) return;
  const int direction=policy.follow_path_direction || std::fabs(signed_speed)<=1.e-3
      ? path.direction : (signed_speed>0?1:-1);
  const double steering = motion.response_reference_valid ? motion.steering_angle : initial_steering_;
  models_[0]={double(direction),0.0,0.0,steering,direction==path.direction};
  count_=1;
  if (motion.measured) {
    if (!std::isfinite(motion.longitudinal_speed) || !std::isfinite(motion.lateral_speed) ||
        !std::isfinite(motion.yaw_rate)) return;
    const double speed=std::hypot(motion.longitudinal_speed,motion.lateral_speed);

    if (speed>0.15 && (!policy.follow_path_direction || path.direction*motion.longitudinal_speed>0)) {
      const double weight=std::clamp((speed-0.15)/0.15,0.0,1.0);
      const double longitudinal=motion.longitudinal_speed/speed;
      models_[1]={double(direction)+weight*(longitudinal-direction),weight*motion.lateral_speed/speed,
          weight*(motion.yaw_rate/speed-longitudinal*std::tan(steering)/policy.wheelbase),
          steering,path.direction*longitudinal>0};

      if (std::fabs(motion.longitudinal_speed)>0.15 && (policy.steer_rate_per_sec>0 || policy.steering_response_time>0)) {
        const double measured_steering=std::atan(policy.wheelbase*
            motion.yaw_rate/motion.longitudinal_speed);
        if (measured_steering>=policy.minimum_steer && measured_steering<=policy.maximum_steer) {
          models_[1].steering=steering+weight*(measured_steering-steering);
          models_[1].yaw_residual=0.0;
        }
      }
      count_=2;
    }
  }
  const double curvature=std::max(std::fabs(std::tan(policy.minimum_steer)),
      std::fabs(std::tan(policy.maximum_steer)))/policy.wheelbase;
  for (int i=0;i<count_;++i) {
    poses_[i]=origin;
    angular_bound_=std::max(angular_bound_,curvature+std::fabs(models_[i].yaw_residual));
  }
  valid_=std::isfinite(angular_bound_);
}

double ApaTrackingMotionPredictor::reference_curvature(double distance) const {
  const auto section=std::upper_bound(sections_.begin(),sections_.end(),distance,
      [](double value,const Section& s) { return value<s.end; });
  return section==sections_.end() ? sections_.back().curvature : section->curvature;
}

double ApaTrackingMotionPredictor::distance_at_time(double time) const {
  const double response=policy_.speed_response_time;
  if (time<=response) return policy_.braking.Speed*std::max(0.0,time);
  return policy_.braking.Speed*response+
      apaMocoBrakingDistanceAtTime(&policy_.braking,time-response);
}

double ApaTrackingMotionPredictor::time_at_distance(double distance) const {
  const double response=policy_.speed_response_time;
  const double travel=policy_.braking.Speed*response;
  if (distance<=travel && policy_.braking.Speed>0) return distance/policy_.braking.Speed;
  return response+apaMocoBrakingTimeAtDistance(&policy_.braking,distance-travel);
}

const std::array<ApaMotionPose,2>& ApaTrackingMotionPredictor::advance(double distance) {
  if (!valid_ || !std::isfinite(distance) || distance<distance_) {
    valid_=false;
    return poses_;
  }
  const double time=time_at_distance(distance);

  const double response_distance = distance_at_time(time+policy_.steering_response_time);
  const double limit=policy_.steer_rate_per_sec>0 ?
      policy_.steer_rate_per_sec*std::max(0.0,time-time_) :
      std::numeric_limits<double>::infinity();
  for (int i=0;i<count_;++i) {
    auto& model=models_[i];
    double target=initial_steering_;
    if (model.follows_reference) {
      const double curvature=reference_curvature(progress_+
          std::fabs(model.longitudinal)*response_distance)+correction_;
      target=std::clamp(std::atan(policy_.wheelbase*curvature),
          policy_.minimum_steer,policy_.maximum_steer);
    }
    const double response = policy_.steering_response_time>0 ?
        (target-model.steering)*(1.0-std::exp(-(time-time_)/policy_.steering_response_time)) :
        target-model.steering;
    const double steering=model.steering+std::clamp(response,-limit,limit);
    const double curvature=(std::tan(model.steering)+std::tan(steering))/(2*policy_.wheelbase);
    const double ds=distance-distance_;
    poses_[i]=AdvanceApaMotionPose(poses_[i],model.longitudinal*ds,model.lateral*ds,
        (model.longitudinal*curvature+model.yaw_residual)*ds);
    model.steering=steering;
  }
  distance_=distance;time_=time;
  return poses_;
}

double ApaTrackingMotionPredictor::path_travel(int model) const {
  return models_[model].follows_reference ? std::fabs(models_[model].longitudinal)*distance_ : 0.0;
}
}
