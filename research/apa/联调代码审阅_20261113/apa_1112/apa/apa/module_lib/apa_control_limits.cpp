#include "apa_control_limits.h"
#include "apaVehicleDefaults.h"

#include <algorithm>
#include <cmath>
#include <limits>

namespace pangu::modules {
namespace {
bool PositiveReal(double value) {
  return std::isfinite(value) && value > 0.0 &&
      value <= std::numeric_limits<Real>::max() && static_cast<Real>(value) > 0.0f;
}

bool DwellCycles(double seconds, Integer* cycles) {
  const double count = std::ceil(seconds / 0.02 - 1.e-6);
  if (!std::isfinite(count) || count < 1.0 || count > std::numeric_limits<Integer>::max())
    return false;
  *cycles = static_cast<Integer>(count);
  return true;
}
}

bool BuildApaControlCalibration(const ApaParams& params,
    ApaMocoParkingControlParam_t* control, std::string* error) {
  apaMocoParkingControlDefaultParam(control);
  *error = "invalid parking control calibration";
  if (params.has_execution_limits()) {
    const auto& limits = params.execution_limits();
    const double values[] = {limits.cusp_position_m(), limits.cusp_yaw_rad(),
        limits.goal_position_m(), limits.goal_yaw_rad(), limits.standstill_speed_mps(),
        limits.cusp_dwell_sec(), limits.goal_dwell_sec(), limits.gear_dwell_sec(),
        limits.reference_distance_m(), limits.reference_yaw_rad()};
    for (double value : values) {
      if (!PositiveReal(value)) {
        *error = "execution limits must fit finite positive Real values";
        return false;
      }
    }
    if (!DwellCycles(limits.cusp_dwell_sec(), &control->Chart.StopDwellCycles) ||
        !DwellCycles(limits.goal_dwell_sec(), &control->Chart.GoalDwellCycles) ||
        !DwellCycles(limits.gear_dwell_sec(), &control->Chart.GearWaitCycles)) {
      *error = "execution dwell times must fit positive Integer cycle counts";
      return false;
    }
    control->Chart.CuspPositionTolerance = static_cast<Real>(limits.cusp_position_m());
    control->Chart.CuspYawTolerance = static_cast<Real>(limits.cusp_yaw_rad());
    control->Chart.GoalPositionTolerance = static_cast<Real>(limits.goal_position_m());
    control->Chart.GoalYawTolerance = static_cast<Real>(limits.goal_yaw_rad());
    control->Chart.StandstillSpeed = static_cast<Real>(limits.standstill_speed_mps());
    control->Longi.StandstillSpeedThreshold = control->Chart.StandstillSpeed;
    control->Path.MaxReferenceDistance = static_cast<Real>(limits.reference_distance_m());
    control->Path.MaxHeadingError = static_cast<Real>(limits.reference_yaw_rad());
  }
  const double optional_values[] = {params.forward_ref_speed_mps(), params.reverse_ref_speed_mps(),
      params.terminal_keep_speed_mps(), params.minimum_tracking_speed_mps(), params.slowdown_distance_m()};
  for (double value : optional_values) {
    if (value != 0.0 && !PositiveReal(std::fabs(value))) {
      *error = "tracking calibration must fit finite Real values";
      return false;
    }
  }
  if (params.forward_ref_speed_mps() > 0.0) {
    control->Longi.ForwardRefSpeed =
        static_cast<Real>(params.forward_ref_speed_mps());
  }
  if (params.reverse_ref_speed_mps() < 0.0) {
    control->Longi.ReverseRefSpeed =
        static_cast<Real>(params.reverse_ref_speed_mps());
  }
  if (params.terminal_keep_speed_mps() > 0.0) {
    control->Longi.TerminalKeepSpeed =
        static_cast<Real>(params.terminal_keep_speed_mps());
    control->Longi.ParallelTerminalKeepSpeed =
        control->Longi.TerminalKeepSpeed;
  }
  const double cusp_speed = params.cusp_approach_speed_mps();
  if (!std::isfinite(cusp_speed) || cusp_speed < 0.0 || cusp_speed > 0.30 ||
      (cusp_speed > 0.0 && cusp_speed <= control->Chart.StandstillSpeed)) {
    *error = "cusp approach speed must exceed standstill and be <= 0.30 m/s";
    return false;
  }
  if (cusp_speed > 0.0) control->Longi.CuspApproachSpeed = static_cast<Real>(cusp_speed);
  if (control->Longi.CuspApproachSpeed <= control->Chart.StandstillSpeed)
    return false;
  const double short_length = params.short_segment_length_m();
  const double short_speed = params.short_segment_speed_mps();
  if (!std::isfinite(short_length) || short_length < 0.0 || short_length > 5.0 ||
      !std::isfinite(short_speed) || short_speed < 0.0 || short_speed > 0.60) {
    *error = "short segment length must be <= 5 m and speed <= 0.60 m/s";
    return false;
  }
  if (short_length > 0.0) control->Longi.ShortSegmentLength = static_cast<Real>(short_length);
  if (short_speed > 0.0) control->Longi.ShortSegmentSpeed = static_cast<Real>(short_speed);
  if (control->Longi.ShortSegmentSpeed < control->Longi.CuspApproachSpeed) {
    *error = "short segment speed must be >= cusp approach speed";
    return false;
  }
  if (params.minimum_tracking_speed_mps() > 0.0) {
    const Real minimum_speed =
        static_cast<Real>(params.minimum_tracking_speed_mps());
    control->Longi.VerticalMinTrackingSpeed = minimum_speed;
    control->Longi.DiagonalMinTrackingSpeed = minimum_speed;
    control->Longi.HeadInMinTrackingSpeed = minimum_speed;
    control->Longi.ParallelMinTrackingSpeed = minimum_speed;
  }
  if (params.slowdown_distance_m() > 0.0) {
    control->Longi.SlowDistThd =
        static_cast<Real>(params.slowdown_distance_m());
  }
  return ApplyApaActuationLimits(params, control, error);
}

bool ApplyApaActuationLimits(const ApaParams& params,
    ApaMocoParkingControlParam_t* control, std::string* error) {
  const double speed = params.target_speed_mps();
  const double accel = params.max_accel_mps2();
  const double steer = params.max_steer_rad();
  if (!std::isfinite(speed) || speed <= control->Chart.StandstillSpeed ||
      !std::isfinite(accel) || !std::isfinite(static_cast<Real>(accel)) || static_cast<Real>(accel) <= 0.0f ||
      !std::isfinite(steer) || static_cast<Real>(steer) <= 0.0f || steer > APA_VEHICLE_MAX_STEER_RAD + 1.e-6) {
    *error = "target_speed_mps/max_accel_mps2/max_steer_rad must be finite positive limits; "
        "speed must exceed standstill and steer must fit the vehicle";
    return false;
  }
  auto& longitudinal = control->Longi;
  longitudinal.SpeedUpperLimit = std::min<double>(longitudinal.SpeedUpperLimit, speed);
  longitudinal.SpeedLowerLimit = std::max<double>(longitudinal.SpeedLowerLimit, -speed);
  longitudinal.ForwardRefSpeed = std::min(longitudinal.ForwardRefSpeed, longitudinal.SpeedUpperLimit);
  longitudinal.ReverseRefSpeed = std::max(longitudinal.ReverseRefSpeed, longitudinal.SpeedLowerLimit);
  longitudinal.AccelerationUpperLimit = std::min<double>(longitudinal.AccelerationUpperLimit, accel);
  longitudinal.AccelerationLowerLimit = std::max<double>(longitudinal.AccelerationLowerLimit, -accel);
  longitudinal.StopDeceleration = std::min<double>(longitudinal.StopDeceleration, accel);
  control->Lat.SteerUpperLimit = std::min<double>(control->Lat.SteerUpperLimit, steer);
  control->Lat.SteerLowerLimit = std::max<double>(control->Lat.SteerLowerLimit, -steer);
  return true;
}

ApaActuationLimits DescribeApaActuationLimits(const ApaMocoParkingControlParam_t& control) {
  ApaActuationLimits limits;
  limits.set_max_abs_target_speed_mps(std::max(control.Longi.SpeedUpperLimit, -control.Longi.SpeedLowerLimit));
  limits.set_max_abs_accel_mps2(std::max(control.Longi.AccelerationUpperLimit, -control.Longi.AccelerationLowerLimit));
  limits.set_max_abs_steer_rad(std::max(control.Lat.SteerUpperLimit, -control.Lat.SteerLowerLimit));
  return limits;
}

}
