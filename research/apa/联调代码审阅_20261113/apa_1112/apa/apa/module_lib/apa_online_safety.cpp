#include "apa_online_safety.h"
#include "apa_scene_limits.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <numeric>
#include <iomanip>
#include <limits>
#include <sstream>
#include <unordered_map>
#include <vector>
extern "C" {
#include "apaMocoPathGeometry.h"
#include "apaVehicleDefaults.h"
#include "apaMocoExecution.h"
#include "apaParkingPathPlanningInternal.h"
#include "apaTrackingPolicy.h"
#include "apaGoalTolerance.h"
#include "apaParkinClearance.h"
}

namespace pangu::modules {
std::vector<ApaScenePoint> ApaObstacleSweptFootprint(const ApaObstacle& obstacle) {
  std::vector<ApaScenePoint> points;
  for (const auto& p : obstacle.footprint()) {
    points.push_back({p.x(), p.y()});
    points.push_back({p.x()+kApaObstaclePredictionSec*obstacle.velocity_x(),
                      p.y()+kApaObstaclePredictionSec*obstacle.velocity_y()});
  }
  std::sort(points.begin(), points.end(), [](auto a, auto b) { return a.x < b.x || (a.x == b.x && a.y < b.y); });
  points.erase(std::unique(points.begin(), points.end(), [](auto a, auto b) { return a.x == b.x && a.y == b.y; }), points.end());
  const auto cross = [](auto a, auto b, auto c) { return (b.x-a.x)*(c.y-a.y)-(b.y-a.y)*(c.x-a.x); };
  std::vector<ApaScenePoint> hull;
  for (auto p : points) {
    while (hull.size() >= 2 && cross(hull[hull.size()-2], hull.back(), p) <= 0.0) hull.pop_back();
    hull.push_back(p);
  }
  const auto lower = hull.size();
  for (int i = static_cast<int>(points.size())-2; i >= 0; --i) {
    while (hull.size() > lower && cross(hull[hull.size()-2], hull.back(), points[i]) <= 0.0) hull.pop_back();
    hull.push_back(points[i]);
  }
  hull.pop_back();
  return hull;
}

namespace {
constexpr double kSceneRadius = 30.0;
constexpr double kSampleStep = 0.01;
using Point = ApaScenePoint;
using Pose = ApaMotionPose;
enum class ClearanceClass { Strict, StaticEnvironment, VirtualBoundary };
enum class ClearanceMode { Planning, Tracking };
struct Obstacle {
  std::string id;
  std::vector<Point> points, swept;
  double vx = 0.0, vy = 0.0;
  ClearanceClass clearance = ClearanceClass::Strict;
};

struct SceneBounds {
  double min_x, min_y, max_x, max_y;
  bool overlaps(const SceneBounds& other) const {
    return max_x >= other.min_x && min_x <= other.max_x &&
           max_y >= other.min_y && min_y <= other.max_y;
  }
  void include(const SceneBounds& other) {
    min_x = std::min(min_x, other.min_x); min_y = std::min(min_y, other.min_y);
    max_x = std::max(max_x, other.max_x); max_y = std::max(max_y, other.max_y);
  }
};
template <typename Points> SceneBounds BoundsOf(const Points& points) {
  SceneBounds bounds{points[0].x, points[0].y, points[0].x, points[0].y};
  for (auto p : points) bounds.include({p.x, p.y, p.x, p.y});
  return bounds;
}

class SceneSpatialIndex {
  struct Node { SceneBounds bounds; size_t begin, end; int left = -1, right = -1; };
  std::vector<SceneBounds> boxes_;
  std::vector<size_t> order_;
  std::vector<Node> nodes_;
  int build(size_t begin, size_t end) {
    SceneBounds bounds = boxes_[order_[begin]];
    for (size_t i = begin+1; i < end; ++i) bounds.include(boxes_[order_[i]]);
    const int node = nodes_.size();
    nodes_.push_back({bounds, begin, end});
    if (end-begin <= 8) return node;
    const bool x_axis = bounds.max_x-bounds.min_x >= bounds.max_y-bounds.min_y;
    const size_t middle = begin+(end-begin)/2;
    std::nth_element(order_.begin()+begin, order_.begin()+middle, order_.begin()+end,
        [&](size_t a, size_t b) {
          const auto& p = boxes_[a]; const auto& q = boxes_[b];
          const double pc = x_axis ? p.min_x/2+p.max_x/2 : p.min_y/2+p.max_y/2;
          const double qc = x_axis ? q.min_x/2+q.max_x/2 : q.min_y/2+q.max_y/2;
          return pc < qc || (pc == qc && a < b);
        });
    const int left = build(begin, middle), right = build(middle, end);
    nodes_[node].left = left; nodes_[node].right = right;
    return node;
  }
  void visit(int index, const SceneBounds& bounds, std::vector<size_t>& found) const {
    const auto& node = nodes_[index];
    if (!node.bounds.overlaps(bounds)) return;
    if (node.left >= 0) {
      visit(node.left, bounds, found); visit(node.right, bounds, found);
    } else {
      for (size_t i = node.begin; i < node.end; ++i)
        if (boxes_[order_[i]].overlaps(bounds)) found.push_back(order_[i]);
    }
  }
 public:
  void reset(std::vector<SceneBounds> boxes) {
    boxes_ = std::move(boxes); nodes_.clear(); order_.resize(boxes_.size());
    std::iota(order_.begin(), order_.end(), 0);
    if (!boxes_.empty()) build(0, boxes_.size());
  }
  std::vector<size_t> query(const SceneBounds& bounds) const {
    std::vector<size_t> found;
    if (!nodes_.empty()) visit(0, bounds, found);
    std::sort(found.begin(), found.end());
    return found;
  }
};
double Dot(Point a, Point b) { return a.x*b.x + a.y*b.y; }

std::array<Point, 8> OctagonBody(double x, double y, double yaw,
    double lf, double lb, double width,
    double front_margin = 0.0, double rear_margin = 0.0, double side_margin = 0.0) {
  const double c = std::cos(yaw), s = std::sin(yaw);
  const double front = lf + front_margin;
  const double back = lb + rear_margin;
  const double side = width/2.0 + side_margin;
  const double f_lon = APA_VEHICLE_FRONT_CHAMFER_LON_M;
  const double f_lat = APA_VEHICLE_FRONT_CHAMFER_LAT_M;
  const double r_lon = APA_VEHICLE_REAR_CHAMFER_LON_M;
  const double r_lat = APA_VEHICLE_REAR_CHAMFER_LAT_M;
  std::array<Point, 8> points{{
      {front - f_lon,  side},
      {front,          side - f_lat},
      {front,         -side + f_lat},
      {front - f_lon, -side},
      {-back + r_lon, -side},
      {-back,         -side + r_lat},
      {-back,          side - r_lat},
      {-back + r_lon,  side},
  }};
  for (auto& p : points) p = {x + p.x*c - p.y*s, y + p.x*s + p.y*c};
  return points;
}

std::array<Point, 8> Body(Pose pose, double front_margin = 0.0, double rear_margin = 0.0, double side_margin = 0.0) {
  return OctagonBody(pose.x, pose.y, pose.yaw,
      APA_VEHICLE_REAR_TO_FRONT_M, APA_VEHICLE_REAR_TO_BACK_M, APA_VEHICLE_WIDTH_M,
      front_margin, rear_margin, side_margin);
}

template <size_t N, typename Polygon>
bool Overlaps(const std::array<Point, N>& body, const Polygon& obstacle, double prediction_sec) {
  const auto separated = [&](Point axis) {
    double body_min = Dot(body[0], axis), body_max = body_min;
    double obs_min = Dot(obstacle.points[0], axis), obs_max = obs_min;
    for (auto p : body) { body_min = std::min(body_min, Dot(p, axis)); body_max = std::max(body_max, Dot(p, axis)); }
    for (auto p : obstacle.points) { obs_min = std::min(obs_min, Dot(p, axis)); obs_max = std::max(obs_max, Dot(p, axis)); }
    const double motion = (obstacle.vx*axis.x + obstacle.vy*axis.y) * prediction_sec;
    obs_min += std::min(0.0, motion); obs_max += std::max(0.0, motion);
    return body_max < obs_min || obs_max < body_min;
  };
  for (size_t i = 0; i < body.size(); ++i) {
    const auto a = body[i], b = body[(i+1) % body.size()];
    if (separated({a.y-b.y, b.x-a.x})) return false;
  }
  for (size_t i = 0; i < obstacle.points.size(); ++i) {
    const auto a = obstacle.points[i], b = obstacle.points[(i+1) % obstacle.points.size()];
    if (separated({a.y-b.y, b.x-a.x})) return false;
  }
  if (separated({-obstacle.vy, obstacle.vx})) return false;
  return true;
}

double PointSegmentDistance(Point p, Point a, Point b) {
  const Point d{b.x-a.x, b.y-a.y};
  const double length2 = Dot(d, d);
  const double t = length2 > 0 ? std::clamp(Dot({p.x-a.x, p.y-a.y}, d)/length2, 0.0, 1.0) : 0.0;
  return std::hypot(p.x-a.x-t*d.x, p.y-a.y-t*d.y);
}

template <size_t N, typename Polygon>
double Gap(const std::array<Point, N>& body, const Polygon& obstacle) {
  if (Overlaps(body, obstacle, 0.0)) return 0.0;
  double gap = std::numeric_limits<double>::infinity();
  for (size_t i = 0; i < obstacle.points.size(); ++i) {
    const auto a = obstacle.points[i], b = obstacle.points[(i+1)%obstacle.points.size()];
    for (size_t j = 0; j < body.size(); ++j) {
      gap = std::min(gap, PointSegmentDistance(body[j], a, b));
      gap = std::min(gap, PointSegmentDistance(a, body[j], body[(j+1)%body.size()]));
    }
  }
  return gap;
}

template <typename Points>
bool BottomObstacle(const VehicleCollisionCheckInput_t& environment, const Points& points) {
  ApaParkinBottom_t bottom;
  if (!apaParkinBottom(&environment,&bottom)) return false;
  double lo=std::numeric_limits<double>::infinity(), hi=-lo;
  for (const auto p : points) {
    if (apaParkinBottomDepth(&bottom,p.x,p.y)>APA_PARKIN_CLEARANCE_EPS_M) return false;
    const auto t=apaParkinBottomAlong(&bottom,p.x,p.y);
    lo=std::min(lo,t); hi=std::max(hi,t);
  }
  return apaParkinBottomSpan(&bottom,lo,hi);
}

template <size_t N, typename Polygon, typename Margins>
bool MarginOverlap(const VehicleCollisionCheckInput_t& environment,
    const std::array<Point,N>& actual, const std::array<Point,N>& envelope,
    const Polygon& polygon, const Margins& margins, bool is_virtual = false) {
  if (!Overlaps(envelope,polygon,0.0)) return false;
  if (!BottomObstacle(environment,polygon.points)) return true;
  const double gap=Gap(actual,polygon);
  const double required = is_virtual
      ? apaParkinVirtualBottomMargin(margins[0],margins[1],margins[2])
      : apaParkinBottomMargin(&environment,margins[0],margins[1],margins[2]);
  return gap<=APA_PARKIN_CLEARANCE_EPS_M || gap+APA_PARKIN_CLEARANCE_EPS_M<required;
}
struct StaticFootprint {
  const std::vector<Point>& points;
  double vx=0.0, vy=0.0;
};

std::vector<Obstacle> Obstacles(const ApaInput& input) {
  std::vector<Obstacle> result;
  for (const auto& item : input.scene().obstacles()) {
    Obstacle obstacle{item.id(), {}, ApaObstacleSweptFootprint(item), item.velocity_x(), item.velocity_y()};

    if (item.id().find(":unknown:")==std::string::npos &&
        item.velocity_x()==0.0 && item.velocity_y()==0.0 &&
        (item.id().rfind("map:",0)==0 || item.id().rfind("offline-map:forbidden:",0)==0 ||
         item.id().rfind("occupancy:",0)==0))
      obstacle.clearance = ClearanceClass::StaticEnvironment;
    for (const auto& point : item.footprint()) obstacle.points.push_back({point.x(), point.y()});
    result.push_back(std::move(obstacle));
  }
  return result;
}

VehicleCollisionCheckInput_t PlanningEnvironment(const ApaInput& input) {
  ApaParkingPathPlanningInput_t request{};
  request.ParkingAction = input.parking_action(); request.TargetSlotKind = input.target_slot_kind();
  request.VehicleX = input.vehicle_x(); request.VehicleY = input.vehicle_y(); request.VehicleYaw = input.vehicle_yaw();
  request.TarX = input.target_x(); request.TarY = input.target_y(); request.TarTheta = input.target_theta();
  request.SltEntr = input.slot_entry();
  request.SltPt1X = input.slot_point1_x(); request.SltPt1Y = input.slot_point1_y();
  request.SltPt2X = input.slot_point2_x(); request.SltPt2Y = input.slot_point2_y();
  request.SltPt3X = input.slot_point3_x(); request.SltPt3Y = input.slot_point3_y();
  request.SltPt4X = input.slot_point4_x(); request.SltPt4Y = input.slot_point4_y();
  request.BoundaryLineNum = input.boundary_line_num();
  request.VirtualBoundaryLineNum = input.virtual_boundary_line_num();
  request.SlotTrackStopperNum = input.slot_track_stopper_num();
  request.BoundarySepPtNum = input.boundary_sep_point_num(); request.StopperPtNum = input.stopper_point_num();
#define COPY_FIELD(member, field) std::copy(input.field().begin(), input.field().end(), request.member)
  COPY_FIELD(BoundaryLineX1, boundary_line_x1); COPY_FIELD(BoundaryLineY1, boundary_line_y1);
  COPY_FIELD(BoundaryLineX2, boundary_line_x2); COPY_FIELD(BoundaryLineY2, boundary_line_y2);
  COPY_FIELD(VirtualBoundaryP1X, virtual_boundary_p1_x); COPY_FIELD(VirtualBoundaryP1Y, virtual_boundary_p1_y);
  COPY_FIELD(VirtualBoundaryP2X, virtual_boundary_p2_x); COPY_FIELD(VirtualBoundaryP2Y, virtual_boundary_p2_y);
  COPY_FIELD(SlotTrackStopperX1, slot_track_stopper_x1); COPY_FIELD(SlotTrackStopperY1, slot_track_stopper_y1);
  COPY_FIELD(SlotTrackStopperX2, slot_track_stopper_x2); COPY_FIELD(SlotTrackStopperY2, slot_track_stopper_y2);
  COPY_FIELD(BoundarySepPtX, boundary_sep_point_x); COPY_FIELD(BoundarySepPtY, boundary_sep_point_y);
  COPY_FIELD(StopperPtX, stopper_point_x); COPY_FIELD(StopperPtY, stopper_point_y);
#undef COPY_FIELD
  ApaParkingPathPlanningParam_t vehicle{};
  vehicle.VehLlF = APA_VEHICLE_REAR_TO_FRONT_M; vehicle.VehLr = APA_VEHICLE_REAR_TO_BACK_M;
  vehicle.VehW = APA_VEHICLE_WIDTH_M;
  VehicleCollisionCheckInput_t environment{};
  apaParkingPathPlanningBuildPathVccInput(&request, &vehicle, &environment);

  environment.VehPosX = request.VehicleX;
  environment.VehPosY = request.VehicleY;
  environment.VehPosTheta = request.VehicleYaw;
  return environment;
}

std::string CollisionId(const ApaInput& input, const VehicleCollisionCheckInput_t& environment,
                        const VehicleCollisionCheckOutput_t& hit) {
  int i = hit.CollisionIndex-1;
  if (i < 0) {

    const bool boundary = hit.CollisionType == VCC_COLLISION_BOUNDARY;
    const bool stopper = hit.CollisionType == VCC_COLLISION_STOPPER;
    const int lines = boundary ? environment.BoundaryLineNum :
        stopper ? environment.SlotTrackStopperNum : environment.VirtualBoundaryLineNum;
    const int points = boundary ? environment.BoundarySepPtsNum : stopper ? environment.SeparateStopperNum : 0;
    int low = 1, high = lines+points;
    while (low < high) {
      const int count = (low+high)/2;
      auto prefix = environment;
      prefix.BoundaryLineNum = boundary ? std::min(count, lines) : 0;
      prefix.BoundarySepPtsNum = boundary ? std::max(0, count-lines) : 0;
      prefix.SlotTrackStopperNum = stopper ? std::min(count, lines) : 0;
      prefix.SeparateStopperNum = stopper ? std::max(0, count-lines) : 0;
      prefix.VirtualBoundaryLineNum = !boundary && !stopper ? count : 0;
      VehicleCollisionCheckOutput_t checked{};
      vehicleCollisionCheck(&prefix, &checked);
      if (checked.IsCollision) high = count;
      else low = count+1;
    }
    i = low-1;
  }
  if (hit.CollisionType == VCC_COLLISION_VIRTUAL) return "json:virtual:" + std::to_string(i);
  if (hit.CollisionType == VCC_COLLISION_STOPPER)
    return i < environment.SlotTrackStopperNum ? "json:slot_stopper:" + std::to_string(i) :
        "json:stopper:" + std::to_string(i-environment.SlotTrackStopperNum);
  if (i >= environment.BoundaryLineNum) return "json:point:" + std::to_string(i-environment.BoundaryLineNum);

  for (int j = 0; j < input.boundary_line_num(); ++j)
    if (static_cast<Real>(input.boundary_line_x1(j)) == environment.BoundaryLineX1[i] &&
        static_cast<Real>(input.boundary_line_y1(j)) == environment.BoundaryLineY1[i] &&
        static_cast<Real>(input.boundary_line_x2(j)) == environment.BoundaryLineX2[i] &&
        static_cast<Real>(input.boundary_line_y2(j)) == environment.BoundaryLineY2[i])
      return "json:boundary:" + std::to_string(j);
  return "json:boundary:" + std::to_string(i);
}

void SetBoundaryPolygon(const std::vector<Point>& polygon, VehicleCollisionCheckInput_t* environment) {
  environment->BoundaryLineNum = polygon.size();
  for (size_t i = 0; i < polygon.size(); ++i) {
    const auto a = polygon[i], b = polygon[(i+1)%polygon.size()];
    environment->BoundaryLineX1[i] = a.x; environment->BoundaryLineY1[i] = a.y;
    environment->BoundaryLineX2[i] = b.x; environment->BoundaryLineY2[i] = b.y;
  }
}

void SetPose(Pose pose, VehicleCollisionCheckInput_t* environment) {
  environment->VehPosX = pose.x; environment->VehPosY = pose.y; environment->VehPosTheta = pose.yaw;
}

bool Contains(const std::vector<Point>& polygon, Point p) {
  bool positive = false, negative = false;
  for (size_t i = 0; i < polygon.size(); ++i) {
    const auto a = polygon[i], b = polygon[(i+1)%polygon.size()];
    const double cross = (b.x-a.x)*(p.y-a.y)-(b.y-a.y)*(p.x-a.x);
    positive |= cross > 0.0; negative |= cross < 0.0;
  }
  return !(positive && negative);
}
}

VehicleCollisionCheckInput_t BuildApaPlanningEnvironment(const ApaInput& input) {
  return PlanningEnvironment(input);
}

double ApaPathProgress(const ::apa::ApaParkingPathPlanningOutputData& path,
    const ApaMocoCtrlChartV2State_t& chart) {
  if (chart.StpNum <= 0 || chart.StpNow < 0 || chart.StpNow >= chart.StpNum ||
      chart.StpNow >= APA_MOCO_V2_MAX_SEG) return 0.0;
  const int start = chart.SegStartIdx[chart.StpNow];
  if (start < 0 || start >= path.pathPointCount) return 0.0;
  double distance = 0.0;
  for (int i = 1; i <= start; ++i)
    distance += std::hypot(path.pathPointX[i] - path.pathPointX[i-1],
                           path.pathPointY[i] - path.pathPointY[i-1]);
  return distance + std::max(0.0, static_cast<double>(chart.Progress));
}

struct ApaPlannerScene::Impl {
  struct Polygon {
    std::string id;
    std::vector<Point> points;
    double vx=0.0, vy=0.0;
  };
  std::vector<Polygon> polygons;
  SceneSpatialIndex index;
  double origin_x = 0.0, origin_y = 0.0;
  bool valid = true;
  size_t queries = 0, hits = 0;
  std::string last_id;

  static Boolean CheckDisk(Real x, Real y, Real radius, void* user_data) {
    auto& self = *static_cast<Impl*>(user_data);
    const Point p{x,y};
    for (size_t i : self.index.query({x-radius,y-radius,x+radius,y+radius})) {
      const auto& polygon = self.polygons[i].points;
      if (Contains(polygon,p)) return true;
      for (size_t j=0;j<polygon.size();++j) {
        const auto a=polygon[j], b=polygon[(j+1)%polygon.size()];
        const double dx=b.x-a.x, dy=b.y-a.y, norm=dx*dx+dy*dy;
        const double t=norm>0 ? std::clamp(((x-a.x)*dx+(y-a.y)*dy)/norm,0.0,1.0) : 0.0;
        if (std::hypot(x-a.x-t*dx,y-a.y-t*dy)<=radius) return true;
      }
    }
    return false;
  }

  static Boolean CheckPose(const VehicleCollisionCheckInput_t* environment,
                           void* user_data) {
    auto& self = *static_cast<Impl*>(user_data);
    ++self.queries;
    const auto blocked = [&](const std::string& id) {
      ++self.hits; self.last_id = id; return true;
    };
    if (!self.valid) return blocked("invalid_scene");
    const double x = environment->VehPosX, y = environment->VehPosY;
    const double yaw = environment->VehPosTheta;
    if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(yaw))
      return blocked("invalid_scene_pose");
    auto actor = *environment;
    actor.BoundaryLineNum = actor.VirtualBoundaryLineNum = 0;
    actor.SlotTrackStopperNum = actor.BoundarySepPtsNum = actor.SeparateStopperNum = 0;
    Real margins[3];
    vehicleCollisionCheckResolveBodyMargins(&actor,margins);
    const double front = actor.VehLF + margins[0];
    const double back = actor.VehLB + margins[1];
    const double half_width = actor.VehW/2.0 + margins[2];
    const double radius = std::max(std::hypot(front, half_width), std::hypot(back, half_width));
    if (std::hypot(x-self.origin_x, y-self.origin_y) + radius > kSceneRadius)
      return blocked("path_exceeds_scene_range");
    const double c = std::cos(yaw), s = std::sin(yaw);
    const auto body = OctagonBody(x, y, yaw, actor.VehLF, actor.VehLB, actor.VehW,
                                  margins[0], margins[1], margins[2]);
    for (size_t i : self.index.query(BoundsOf(body))) {
      const auto& polygon = self.polygons[i];
      if(actor.CheckType==VCC_CHECK_TYPE_PLANNING) {
        const auto actual = OctagonBody(x, y, yaw, actor.VehLF, actor.VehLB, actor.VehW);
        if(MarginOverlap(actor,actual,body,polygon,margins)) return blocked(polygon.id);
        continue;
      }
      SetBoundaryPolygon(polygon.points, &actor);
      VehicleCollisionCheckOutput_t native{};
      vehicleCollisionCheck(&actor, &native);
      const double center = (actor.VehLF-actor.VehLB)/2.0;
      if (native.IsCollision || Contains(polygon.points, {x+center*c, y+center*s}))
        return blocked(polygon.id);
    }
    return false;
  }
};

ApaPlannerScene::ApaPlannerScene(const ApaInput& input) : impl_(std::make_unique<Impl>()) {
  impl_->origin_x = input.vehicle_x(); impl_->origin_y = input.vehicle_y();
  impl_->valid = input.has_scene() && input.scene().obstacles_size() <= kApaSceneMaxObstacles;
  for (const auto& obstacle : input.scene().obstacles()) {
    if (obstacle.footprint_size() < 3 || obstacle.footprint_size() > VCC_MAX_BOUNDARY_LINES ||
        !std::isfinite(obstacle.velocity_x()) || !std::isfinite(obstacle.velocity_y())) {
      impl_->valid = false; break;
    }
    for (const auto& p : obstacle.footprint())
      if (!std::isfinite(p.x()) || !std::isfinite(p.y())) impl_->valid = false;
    if (!impl_->valid) break;
    auto hull = ApaObstacleSweptFootprint(obstacle);
    if (hull.size() < 3 || hull.size() > VCC_MAX_BOUNDARY_LINES) {
      impl_->valid = false; break;
    }
    Impl::Polygon polygon{obstacle.id(), std::move(hull)};
    impl_->polygons.push_back(std::move(polygon));
  }
  std::vector<SceneBounds> bounds;
  for (const auto& polygon : impl_->polygons)
    bounds.push_back(BoundsOf(polygon.points));
  impl_->index.reset(std::move(bounds));
}
ApaPlannerScene::~ApaPlannerScene() = default;
ApaPlanningSceneQuery_t ApaPlannerScene::query() {
  ApaPlanningSceneQuery_t query{};
  query.CheckPose = &Impl::CheckPose;
  query.UserData = impl_.get();
  query.CheckDisk = &Impl::CheckDisk;
  return query;
}
size_t ApaPlannerScene::pose_queries() const { return impl_->queries; }
size_t ApaPlannerScene::obstacle_hits() const { return impl_->hits; }
const std::string& ApaPlannerScene::last_obstacle_id() const { return impl_->last_id; }

namespace {
constexpr double kGeometryTolerance = 1.e-5;
using ClearanceState = std::unordered_map<std::string, double>;

std::string GeometryKey(const Obstacle& obstacle) {
  std::ostringstream out;
  out << obstacle.id << ':' << std::hexfloat << obstacle.vx << ':' << obstacle.vy;
  for (auto p : obstacle.points) out << ':' << p.x << ',' << p.y;
  return out.str();
}

ClearanceState Limits(const ApaClearanceRecovery* recovery) {
  ClearanceState result;
  if (recovery && recovery->info.enabled)
    for (const auto& item : recovery->limits) result.emplace(item.geometry_key, item.minimum_gap);
  return result;
}

void SaveLimits(const ClearanceState& state, ApaClearanceRecovery* recovery) {
  recovery->limits.clear();
  for (const auto& [key, gap] : state) recovery->limits.push_back({key, gap});
}

struct SafetyGeometry {
  std::vector<Obstacle> constraints;
  SceneSpatialIndex index;
  void Add(std::string id, std::vector<Point> points, ClearanceClass clearance) {
    constraints.push_back({std::move(id), points, std::move(points), 0, 0, clearance});
  }
  explicit SafetyGeometry(const ApaInput& input) {
    const auto environment_ = PlanningEnvironment(input);

    for (int i=0; i<environment_.VirtualBoundaryLineNum; ++i)
      Add("json:virtual:"+std::to_string(i), {{environment_.VirtualBoundaryLineX1[i], environment_.VirtualBoundaryLineY1[i]},
          {environment_.VirtualBoundaryLineX2[i], environment_.VirtualBoundaryLineY2[i]}},ClearanceClass::VirtualBoundary);
    for (int i=0; i<environment_.BoundaryLineNum; ++i)
      Add("json:boundary:"+std::to_string(i), {{environment_.BoundaryLineX1[i], environment_.BoundaryLineY1[i]},
          {environment_.BoundaryLineX2[i], environment_.BoundaryLineY2[i]}},ClearanceClass::StaticEnvironment);
    for (int i=0; i<environment_.BoundarySepPtsNum; ++i)
      Add("json:point:"+std::to_string(i), {{environment_.BoundarySepPtsX[i], environment_.BoundarySepPtsY[i]}},
          ClearanceClass::StaticEnvironment);
    for (auto& obstacle : Obstacles(input)) constraints.push_back(std::move(obstacle));
    std::vector<SceneBounds> bounds;
    for (const auto& item : constraints) {
      bounds.push_back(BoundsOf(item.swept));
    }
    index.reset(std::move(bounds));
  }
};

class SafetyScene {
  const ApaInput& input_;
  VehicleCollisionCheckInput_t environment_;
  std::unique_ptr<SafetyGeometry> owned_geometry_;
  const SafetyGeometry& geometry_;
 public:
  double ExecutionFloor(size_t i, ClearanceMode mode) const {
    if (mode == ClearanceMode::Tracking &&
        geometry_.constraints[i].clearance == ClearanceClass::VirtualBoundary)
      return APA_TRACKING_VIRTUAL_CLEARANCE_M;
    if (mode == ClearanceMode::Tracking &&
        geometry_.constraints[i].clearance == ClearanceClass::StaticEnvironment)
      return APA_TRACKING_STATIC_MAP_CLEARANCE_M;
    return std::numeric_limits<double>::infinity();
  }
  explicit SafetyScene(const ApaInput& input) : input_(input),
      environment_(PlanningEnvironment(input)),
      owned_geometry_(std::make_unique<SafetyGeometry>(input)), geometry_(*owned_geometry_) {}
  SafetyScene(const ApaInput& input, const SafetyGeometry& geometry) : input_(input),
      environment_(PlanningEnvironment(input)), geometry_(geometry) {}

  std::array<double, 3> Margins(Pose pose) const {
    auto native = environment_;
    SetPose(pose, &native);
    Real margins[3];
    vehicleCollisionCheckResolveBodyMargins(&native,margins);
    const double extra = apaPlanningPhysicalExtraReserve(&native, APA_PLANNING_TRACKING_RESERVE_M);
    return {margins[0]+extra, margins[1]+extra, margins[2]+extra};
  }

  std::array<double, 3> VirtualMargins(Pose pose) const {
    auto native = environment_;
    SetPose(pose, &native);
    Real margins[3];
    vehicleCollisionCheckResolveVirtualMargins(&native, margins);
    return {margins[0], margins[1], margins[2]};
  }

  ApaOnlineSafety Block(const char* reason, const std::string& id) const {
    ApaOnlineSafety result;
    result.set_scene_version(input_.scene().version());
    result.set_blocked(true); result.set_reason(reason); result.set_obstacle_id(id);
    return result;
  }

  double RecoveryJoinGap(Pose pose) const {
    const double target=APA_CLEARANCE_RECOVERY_TARGET_GAP_M;
    const auto body=Body(pose);
    const auto nearby=Body(pose,target,target,target);
    double gap=target;
    for (size_t i : geometry_.index.query(BoundsOf(nearby))) {
      if (!environment_.ParkingOutFlag &&
          geometry_.constraints[i].clearance == ClearanceClass::VirtualBoundary)
        continue;
      gap=std::min(gap,Gap(body,StaticFootprint{geometry_.constraints[i].swept}));
    }
    return gap;
  }

  ApaOnlineSafety Wheels(Pose pose, bool planning, const char* reason) const {
    auto wheel = planning ? environment_ : apaCollisionBodyEnvelope(&environment_, 0.0f);
    SetPose(pose, &wheel);
    wheel.BoundaryLineNum = wheel.VirtualBoundaryLineNum = wheel.BoundarySepPtsNum = 0;
    VehicleCollisionCheckOutput_t checked{};
    vehicleCollisionCheck(&wheel, &checked);
    if (checked.IsCollision) return Block(reason, CollisionId(input_, wheel, checked));
    ApaOnlineSafety result; result.set_scene_version(input_.scene().version()); return result;
  }

  void Observe(ApaCandidateSafetyObservation* observation, Pose pose,
      double distance, const ApaOnlineSafety& result, int model = -1,
      ClearanceMode mode = ClearanceMode::Planning) const {
    if (!observation || !result.blocked() || observation->pose_valid) return;
    observation->pose_valid = true;
    observation->x = pose.x; observation->y = pose.y; observation->yaw = pose.yaw;
    observation->sample_distance_m = distance;
    observation->stage = result.reason(); observation->prediction_model = model;
    observation->margins_m = Margins(pose);
    observation->obstacle_kind = "native_constraint";
    for (size_t i=0;i<geometry_.constraints.size();++i) {
      const auto& item=geometry_.constraints[i];
      if (item.id != result.obstacle_id()) continue;
      if (item.clearance == ClearanceClass::VirtualBoundary)
        observation->margins_m = VirtualMargins(pose);
      if (std::isfinite(ExecutionFloor(i,mode))) observation->execution_gap_floor_m=ExecutionFloor(i,mode);
      observation->obstacle_kind = item.clearance == ClearanceClass::VirtualBoundary ? "virtual_line" :
          item.id.rfind("json:boundary:",0)==0 ? "boundary_line" :
          item.id.rfind("json:point:",0)==0 ? "boundary_point" :
          item.id.find(":unknown:")!=std::string::npos ? "unknown_region" :
          item.id.rfind("occupancy:",0)==0 ? "parked_vehicle" :
          std::hypot(item.vx,item.vy)>1.e-4 ? "moving_obstacle" : "static_obstacle";
      observation->current_gap_m = Gap(Body({input_.vehicle_x(),input_.vehicle_y(),input_.vehicle_yaw()}),item);
      observation->checked_gap_m = Gap(Body(pose),item);
      observation->checked_swept_gap_m = Gap(Body(pose),StaticFootprint{item.swept});
      break;
    }
  }

  ApaOnlineSafety BodyCheck(Pose pose, double prediction = 0.0, const char* future_reason = nullptr,
      bool ignore_virtual = false, const ClearanceState* restoration = nullptr) const {
    auto result = Wheels(pose, false, future_reason ? future_reason : "body_blocked");
    if (result.blocked()) return result;
    const auto body = Body(pose);
    for (size_t i : geometry_.index.query(BoundsOf(body))) {
      const auto& item = geometry_.constraints[i];
      if (item.clearance == ClearanceClass::VirtualBoundary) {
        if (ignore_virtual) continue;
        if (restoration) {
          const auto allowed = restoration->find(GeometryKey(item));
          if (allowed != restoration->end() && allowed->second < 0.0) continue;
        }
      }
      if (Overlaps(body, item, prediction))
        return Block(future_reason ? future_reason : item.id.rfind("json:virtual:",0)==0 ?
            "line_contact" : "body_blocked", item.id);
    }
    return result;
  }

  ApaOnlineSafety PlanningCheck(Pose pose, double prediction, const char* reason,
      ClearanceState* restoration = nullptr, ClearanceMode mode = ClearanceMode::Planning,
      bool ignore_virtual = false) const {
    auto result = BodyCheck(pose, prediction, reason, ignore_virtual, restoration);
    if (result.blocked()) return result;
    result = Wheels(pose, true, reason);
    if (result.blocked()) return result;
    const auto margins = Margins(pose);
    const auto virt_margins = VirtualMargins(pose);
    const auto envelope = Body(pose, margins[0], margins[1], margins[2]);
    const auto virt_envelope = Body(pose, virt_margins[0], virt_margins[1], virt_margins[2]);
    const double radius = std::max(std::hypot(APA_VEHICLE_REAR_TO_FRONT_M+margins[0], APA_VEHICLE_WIDTH_M/2+margins[2]),
        std::hypot(APA_VEHICLE_REAR_TO_BACK_M+margins[1], APA_VEHICLE_WIDTH_M/2+margins[2]));
    if (std::hypot(pose.x-input_.vehicle_x(),pose.y-input_.vehicle_y())+radius>kSceneRadius)
      return Block("path_exceeds_scene_range", "");
    const auto body = Body(pose);
    auto query_bounds = BoundsOf(envelope);
    query_bounds.include(BoundsOf(virt_envelope));
    for (size_t i : geometry_.index.query(query_bounds)) {
      const auto& item = geometry_.constraints[i];
      const bool is_virtual = item.clearance == ClearanceClass::VirtualBoundary;
      if (is_virtual && ignore_virtual) continue;
      const auto& active_envelope = is_virtual ? virt_envelope : envelope;
      const auto& active_margins = is_virtual ? virt_margins : margins;
      if (!MarginOverlap(environment_,body,active_envelope,
          StaticFootprint{prediction>0.0 ? item.swept : item.points},active_margins,is_virtual)) continue;
      const double gap = Gap(body, item);
      if (gap>ExecutionFloor(i,mode)+kGeometryTolerance)
        continue;

      const auto allowed = restoration ? restoration->find(GeometryKey(item)) : ClearanceState::const_iterator{};
      if (!restoration || allowed == restoration->end() || std::hypot(item.vx,item.vy)>1.e-4)
        return Block(reason, item.id);
      if (is_virtual && allowed->second < 0.0) continue;
      if (gap <= APA_CLEARANCE_RECOVERY_MIN_GAP_M+kGeometryTolerance ||
          gap+kGeometryTolerance < allowed->second-APA_CLEARANCE_RECOVERY_GAP_SWING_M)
        return Block("clearance_recovery_deviation", item.id);
    }

    return result;
  }

  ApaClearanceRecovery Prepare() const {
    ApaClearanceRecovery result;
    const Pose pose{input_.vehicle_x(), input_.vehicle_y(), input_.vehicle_yaw()};
    const bool parkin = !environment_.ParkingOutFlag;
    if (BodyCheck(pose, kApaObstaclePredictionSec, nullptr, parkin).blocked() ||
        Wheels(pose,true,"planning_clearance").blocked()) return result;
    const auto margins = Margins(pose);
    const auto virt_margins = VirtualMargins(pose);
    const auto envelope = Body(pose,margins[0],margins[1],margins[2]);
    const auto virt_envelope = Body(pose,virt_margins[0],virt_margins[1],virt_margins[2]);
    const auto body = Body(pose);
    auto query_bounds = BoundsOf(envelope);
    query_bounds.include(BoundsOf(virt_envelope));
    for (size_t i : geometry_.index.query(query_bounds)) {
      const auto& item = geometry_.constraints[i];
      const bool is_virtual = item.clearance == ClearanceClass::VirtualBoundary;
      const auto& active_envelope = is_virtual ? virt_envelope : envelope;
      const auto& active_margins = is_virtual ? virt_margins : margins;
      if (!MarginOverlap(environment_,body,active_envelope,StaticFootprint{item.swept},active_margins,is_virtual))
        continue;
      if (is_virtual) {
        result.limits.push_back({GeometryKey(item),-1.0});
        continue;
      }
      const double gap = Gap(body,item);
      if (std::hypot(item.vx,item.vy)>1.e-4 ||
          gap<=APA_CLEARANCE_RECOVERY_MIN_GAP_M+kGeometryTolerance) return {};
      result.limits.push_back({GeometryKey(item),gap});
    }
    result.info.enabled = !result.limits.empty();
    result.info.origin_x=pose.x; result.info.origin_y=pose.y; result.info.origin_yaw=pose.yaw;
    return result;
  }
};

bool ValidRecovery(const ApaClearanceRecovery* recovery) {
  if (!recovery || !recovery->info.enabled) return true;
  const auto& r=recovery->info;
  if (!std::isfinite(r.prefix_length) || r.prefix_length<=0 ||
      r.prefix_length>APA_CLEARANCE_RECOVERY_MAX_LENGTH_M+1.e-4 ||
      !std::isfinite(r.origin_x) || !std::isfinite(r.origin_y) || !std::isfinite(r.origin_yaw)) return false;
  for (const auto& limit:recovery->limits) {
    if (limit.geometry_key.empty() || !std::isfinite(limit.minimum_gap)) return false;
    if (limit.geometry_key.rfind("json:virtual:", 0) == 0 && limit.minimum_gap == -1.0) continue;
    if (limit.minimum_gap<=APA_CLEARANCE_RECOVERY_MIN_GAP_M+kGeometryTolerance) return false;
  }
  return true;
}
}

struct ApaSafetyScene::Impl : SafetyGeometry {
  explicit Impl(const ApaInput& input) : SafetyGeometry(input) {}
};
ApaSafetyScene::ApaSafetyScene(const ApaInput& input) : impl_(std::make_unique<Impl>(input)) {}
ApaSafetyScene::~ApaSafetyScene() = default;

ApaOnlineSafety CheckApaBody(const ApaInput& input) {
  return SafetyScene(input).BodyCheck({input.vehicle_x(), input.vehicle_y(), input.vehicle_yaw()},
      0.0, nullptr, input.parking_action() == APA_SCENARIO_PARKING_ACTION_PARKIN);
}

ApaOnlineSafety CheckApaPlanningStart(const ApaInput& input) {
  return ApaSafetyScene(input).CheckPlanningStart(input);
}
ApaOnlineSafety ApaSafetyScene::CheckPlanningStart(const ApaInput& input) const {
  SafetyScene scene(input,*impl_);
  const Pose pose{input.vehicle_x(),input.vehicle_y(),input.vehicle_yaw()};
  const auto body=scene.BodyCheck(pose);
  return body.blocked()?body:scene.PlanningCheck(pose,kApaObstaclePredictionSec,"planning_clearance");
}

ApaClearanceRecovery PrepareApaClearanceRecovery(const ApaInput& input) {
  return SafetyScene(input).Prepare();
}

namespace {
ApaMotionPathView MotionPathView(const ::apa::ApaParkingPathPlanningOutputData& path,
    const ApaMocoCtrlChartV2State_t& chart) {
  if (chart.StpNum<=0 || chart.StpNum>APA_MOCO_V2_MAX_SEG ||
      chart.StpNow<0 || chart.StpNow>=chart.StpNum) return {};
  const int start=chart.SegStartIdx[chart.StpNow], end=chart.SegEndIdx[chart.StpNow];
  if (start<0 || end<=start || end>=path.pathPointCount || path.pathPointCount>PATH_PT_MAX) return {};
  return {path.pathPointX.data()+start,path.pathPointY.data()+start,
      path.pathPointTheta.data()+start,path.pathPointOdometer.data()+start,
      end-start+1,int(chart.SegGoDir[chart.StpNow]),chart.Progress};
}

ClearanceMode TrackingMode(const ApaInput& input,
    const ::apa::ApaParkingPathPlanningOutputData& path,
    const ApaMocoCtrlChartV2State_t& chart) {

  if (chart.Phase==APA_MOCO_FAULT) return ClearanceMode::Planning;
  const auto view=MotionPathView(path,chart);
  if (view.count<2 || !std::isfinite(view.progress)) return ClearanceMode::Planning;
  Integer index=0; Real x,y,yaw;
  apaMocoPathAtDistance(view.x,view.y,view.yaw,view.count,view.progress,&index,&x,&y,&yaw);
  const double error=input.vehicle_yaw()-yaw;
  if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(error) ||
      std::hypot(input.vehicle_x()-x,input.vehicle_y()-y)>APA_TRACKING_REFERENCE_ERROR_M ||
      std::fabs(std::atan2(std::sin(error),std::cos(error)))>APA_TRACKING_HEADING_ERROR_RAD)
    return ClearanceMode::Planning;
  return ClearanceMode::Tracking;
}

ApaAvailableMotion CheckTrackingMotion(const SafetyScene& scene, const ApaInput& input,
    const ::apa::ApaParkingPathPlanningOutputData& path,
    const ApaMocoCtrlChartV2State_t& chart, const ApaMotionPredictionPolicy& policy,
    const ApaMotionPrediction& motion, const ApaClearanceRecovery* recovery, double horizon,
    ApaCandidateSafetyObservation* observation = nullptr) {
  const double progress=ApaPathProgress(path,chart);
  const bool restoring=recovery && recovery->info.enabled;
  const bool ignore_virtual = input.parking_action() == APA_SCENARIO_PARKING_ACTION_PARKIN;
  const auto mode=TrackingMode(input,path,chart);
  const Pose origin{input.vehicle_x(),input.vehicle_y(),input.vehicle_yaw()};
  ApaAvailableMotion available;
  ApaTrackingMotionPredictor predictor(origin,MotionPathView(path,chart),policy,motion,input.vehicle_speed());
  if (!predictor.valid() || !std::isfinite(horizon) || horizon<0 || horizon>kSceneRadius) {
    available.obstacle=scene.Block("invalid_motion_prediction",""); return available;
  }
  if (!ValidRecovery(recovery)) {
    available.obstacle=scene.Block("invalid_clearance_recovery",""); return available;
  }
  const auto check=[&](Pose pose,double distance,ClearanceState* limits) {
    const bool prefix=restoring && progress+distance<recovery->info.prefix_length-1.e-5;
    if (prefix && std::hypot(pose.x-recovery->info.origin_x,pose.y-recovery->info.origin_y)>
        APA_CLEARANCE_RECOVERY_MAX_LENGTH_M+1.e-4)
      return scene.Block("clearance_recovery_deviation","");
    return scene.PlanningCheck(pose,kApaObstaclePredictionSec,"clearance_predicted",
        prefix?limits:nullptr,mode,ignore_virtual);
  };
  auto initial=Limits(recovery);
  available.obstacle=check(origin,0,&initial);
  scene.Observe(observation,origin,0,available.obstacle,-1,mode);
  if (available.obstacle.blocked()) return available;
  std::array<ClearanceState,2> limits{initial,initial};
  const double corner=std::hypot(APA_VEHICLE_REAR_TO_FRONT_M+.20,APA_VEHICLE_WIDTH_M/2+.15);
  const double steps=std::ceil(horizon*(1+corner*predictor.maximum_angular_motion())/kSampleStep);
  if (!std::isfinite(steps) || steps>std::numeric_limits<int>::max()) {
    available.obstacle=scene.Block("invalid_motion_prediction","");return available;
  }
  const int samples=std::max(1,int(steps));
  double previous=0;
  for (int i=1;i<=samples;++i) {
    const double distance=horizon*i/samples;
    const auto& poses=predictor.advance(distance);
    for (int model=0;model<predictor.model_count();++model) {
      if (!predictor.valid() || !std::isfinite(poses[model].x) ||
          !std::isfinite(poses[model].y) || !std::isfinite(poses[model].yaw)) {
        available.obstacle=scene.Block("invalid_motion_prediction","");return available;
      }
      const bool prefix=restoring && progress+predictor.path_travel(model)<recovery->info.prefix_length-1.e-5;
      auto obstacle=scene.BodyCheck(poses[model],kApaObstaclePredictionSec,"motion_blocked",
          ignore_virtual,prefix?&limits[model]:nullptr);
      if (!obstacle.blocked()) obstacle=check(poses[model],predictor.path_travel(model),&limits[model]);
      if (obstacle.blocked()) {
        scene.Observe(observation,poses[model],distance,obstacle,model,mode);
        available.distance=previous;available.obstacle=std::move(obstacle);
        available.obstacle.set_checked_distance_m(distance);return available;
      }
    }
    previous=distance;
  }
  available.distance=horizon;
  return available;
}
}

ApaAvailableMotion CheckApaAvailableMotion(const ApaInput& input,
    const ::apa::ApaParkingPathPlanningOutputData& path,
    const ApaMocoCtrlChartV2State_t& chart, const ApaMotionPredictionPolicy& policy,
    const ApaMotionPrediction& motion, const ApaClearanceRecovery* recovery, double horizon) {
  return ApaSafetyScene(input).CheckAvailableMotion(input,path,chart,policy,motion,recovery,horizon);
}
ApaAvailableMotion ApaSafetyScene::CheckAvailableMotion(const ApaInput& input,
    const ::apa::ApaParkingPathPlanningOutputData& path,
    const ApaMocoCtrlChartV2State_t& chart, const ApaMotionPredictionPolicy& policy,
    const ApaMotionPrediction& motion, const ApaClearanceRecovery* recovery, double horizon,
    ApaCandidateSafetyObservation* observation) const {
  if (observation) *observation = {};

  const double budget=policy.braking.Speed*policy.speed_response_time+policy.braking.Distance;
  const auto view=MotionPathView(path,chart);
  double length=0;
  for (int i=0;i+1<view.count;++i) length+=std::hypot(double(view.x[i+1])-view.x[i],
      double(view.y[i+1])-view.y[i]);
  const double remaining=std::max(0.0,length-view.progress);

  return CheckTrackingMotion(SafetyScene(input,*impl_),input,path,chart,policy,motion,recovery,
      std::min({horizon,budget,remaining}),observation);
}

ApaOnlineSafety ApaSafetyScene::CheckTrackingPose(const ApaInput& input,
    const ::apa::ApaParkingPathPlanningOutputData& path,
    const ApaMocoCtrlChartV2State_t& chart) const {
  return SafetyScene(input,*impl_).PlanningCheck(
      {input.vehicle_x(),input.vehicle_y(),input.vehicle_yaw()},
      kApaObstaclePredictionSec,"clearance_deficit",nullptr,TrackingMode(input,path,chart));
}

bool BuildApaClearancePrefix(const ApaInput& input,const ApaClearanceRecovery& recovery,
    double curvature,int direction,::apa::ApaParkingPathPlanningOutputData* prefix,
    bool* target_reached, double* join_gap) {
  *prefix={};
  if (target_reached) *target_reached=false;
  if (join_gap) *join_gap=0;
  if (!recovery.info.enabled || recovery.limits.empty() || (direction!=1 && direction!=-1) || !std::isfinite(curvature)) return false;
  SafetyScene scene(input);
  const Pose origin{input.vehicle_x(),input.vehicle_y(),input.vehicle_yaw()};
  auto state=Limits(&recovery);
  if (scene.BodyCheck(origin,0.0,nullptr,false,&state).blocked() ||
      scene.PlanningCheck(origin,kApaObstaclePredictionSec,"planning_clearance",&state).blocked()) return false;
  ::apa::ApaParkingPathPlanningOutputData candidate{};
  double best_gap=-1;
  const auto append=[&](Pose pose) {
    int i=candidate.pathPointCount++;
    candidate.pathPointX[i]=pose.x; candidate.pathPointY[i]=pose.y; candidate.pathPointTheta[i]=pose.yaw;
    if(i) {
      const double ds=std::hypot(candidate.pathPointX[i]-candidate.pathPointX[i-1],candidate.pathPointY[i]-candidate.pathPointY[i-1]);
      candidate.pathPointOdometer[i-1]=direction*ds; candidate.pathLength+=ds;
    }
  };
  append(origin);
  constexpr double step_length=.04;
  const int max_steps=int(APA_CLEARANCE_RECOVERY_MAX_LENGTH_M/step_length+1.e-6);
  const double radius=std::hypot(APA_VEHICLE_REAR_TO_FRONT_M+.20,APA_VEHICLE_WIDTH_M/2+.15);
  const int samples=std::max(1,int(std::ceil(step_length*(1+radius*std::fabs(curvature))/kSampleStep)));
  for(int step=1;step<=max_steps;++step) {
    Pose pose=origin;
    for(int i=1;i<=samples;++i) {
      const double distance=step_length*(step-1+double(i)/samples);
      pose=AdvanceApaMotionPose(origin,direction*distance,0,direction*distance*curvature);
      if(scene.PlanningCheck(pose,kApaObstaclePredictionSec,"planning_clearance",&state).blocked()) return prefix->planOk;
    }
    append(pose);
    if(!scene.PlanningCheck(pose,kApaObstaclePredictionSec,"planning_clearance").blocked()) {
      const double gap=scene.RecoveryJoinGap(pose);
      const bool preferred=gap+kGeometryTolerance>=APA_CLEARANCE_RECOVERY_TARGET_GAP_M;
      if (!prefix->planOk || preferred || gap>best_gap+kGeometryTolerance) {
        *prefix=candidate;
        prefix->planOk=true; prefix->segmentCount=1; prefix->maneuverStepCount=1;
        best_gap=gap;
        if (join_gap) *join_gap=gap;
      }
      if (preferred) {
        if (target_reached) *target_reached=true;
        return true;
      }
    }
  }
  return prefix->planOk;
}

static ApaOnlineSafety CheckPath(const SafetyScene& scene, const ApaInput& input,
    const ::apa::ApaParkingPathPlanningOutputData* path,
    const ApaMocoCtrlChartV2State_t& chart,double stop_distance,double preview_speed,bool whole_path,
    const ApaClearanceRecovery* recovery,const ApaMotionPrediction* motion=nullptr,
    const ApaMotionPredictionPolicy* prediction_policy=nullptr,
    ApaClearanceRecovery* observed_recovery=nullptr,
    ApaCandidateSafetyObservation* observation=nullptr) {
  const bool recovering=recovery && recovery->info.enabled;
  const double origin_progress=path && !whole_path?ApaPathProgress(*path,chart):0;
  const bool prefix=recovering && origin_progress<recovery->info.prefix_length-1.e-5;

  const bool ignore_virtual = !whole_path &&
      input.parking_action() == APA_SCENARIO_PARKING_ACTION_PARKIN;
  const auto mode=!whole_path && path ? TrackingMode(input,*path,chart) : ClearanceMode::Planning;
  const bool valid_recovery = ValidRecovery(recovery);
  auto state = valid_recovery ? Limits(recovery) : ClearanceState{};
  Pose previous{input.vehicle_x(),input.vehicle_y(),input.vehicle_yaw()};
  auto result=scene.BodyCheck(previous,0.0,nullptr,ignore_virtual,prefix&&valid_recovery?&state:nullptr);
  scene.Observe(observation,previous,0,result,-1,mode);
  if(result.blocked() || (!whole_path && chart.Phase==APA_MOCO_DONE)) return result;
  if(!valid_recovery) return scene.Block("invalid_clearance_recovery","");
  const auto check=[&](Pose pose,double distance,ClearanceState* limits,const char* reason) {
    const bool prefix=recovering && distance<recovery->info.prefix_length-1.e-5;
    if(prefix && std::hypot(pose.x-recovery->info.origin_x,pose.y-recovery->info.origin_y)>
        APA_CLEARANCE_RECOVERY_MAX_LENGTH_M+1.e-4) return scene.Block("clearance_recovery_deviation","");
    return scene.PlanningCheck(pose,kApaObstaclePredictionSec,reason,
        prefix?limits:nullptr,mode,ignore_virtual);
  };
  result=scene.BodyCheck(previous,kApaObstaclePredictionSec,"body_predicted",
      ignore_virtual,prefix?&state:nullptr);
  scene.Observe(observation,previous,origin_progress,result,-1,mode);
  if(result.blocked()) return result;
  result=check(previous,origin_progress,&state,whole_path?"planning_clearance":"clearance_deficit");
  scene.Observe(observation,previous,origin_progress,result,-1,mode);
  if(result.blocked()) return result;
  if(observed_recovery && recovering) SaveLimits(state,observed_recovery);
  if(!path || chart.StpNum==0) return result;

  if (!whole_path && motion) {
    if (!prediction_policy) return scene.Block("invalid_motion_prediction","");
    const double speed=std::max(std::fabs(input.vehicle_speed()),motion->measured ?
        std::hypot(motion->longitudinal_speed,motion->lateral_speed) : 0.0);

    if (speed>1.e-3) {
      const auto predicted=CheckTrackingMotion(scene,input,*path,chart,*prediction_policy,
          *motion,recovery,stop_distance,observation);
      if (predicted.obstacle.blocked()) return predicted.obstacle;
    }
  }

  if (chart.StpNum<0 || chart.StpNum>APA_MOCO_V2_MAX_SEG || chart.StpNow<0 ||
      chart.StpNow>=chart.StpNum || chart.SegStartIdx[chart.StpNow]<0 ||
      chart.SegEndIdx[chart.StpNow]>=path->pathPointCount ||
      chart.SegEndIdx[chart.StpNow]<=chart.SegStartIdx[chart.StpNow])
    return scene.Block("invalid_motion_prediction","");
  const int segment=chart.StpNow,start=chart.SegStartIdx[segment],end=chart.SegEndIdx[segment];
  const auto* x=path->pathPointX.data()+start;
  const auto* y=path->pathPointY.data()+start;
  const auto* yaw=path->pathPointTheta.data()+start;
  double length=0;
  std::vector<double> vertices;
  for(int i=1;i<=end-start;++i) {
    length+=std::hypot(x[i]-x[i-1],y[i]-y[i-1]);
    vertices.push_back(length-chart.Progress);
  }
  const double horizon=std::max({.5,stop_distance,preview_speed*kApaObstaclePredictionSec});
  const double preview=whole_path?length:std::min(horizon,std::max(0.0,length-chart.Progress));
  if(!std::isfinite(preview) || (!whole_path && horizon+std::hypot(APA_VEHICLE_REAR_TO_FRONT_M,APA_VEHICLE_WIDTH_M/2)>kSceneRadius))
    return scene.Block("stopping_horizon_exceeds_scene_range","");
  double previous_distance=origin_progress;
  size_t vertex=0;
  for(double distance=0;;) {
    Integer index=0; Real px,py,heading;
    apaMocoPathAtDistance(x,y,yaw,end-start+1,chart.Progress+distance,&index,&px,&py,&heading);
    const Pose next{px,py,heading};
    const double dyaw=std::atan2(std::sin(next.yaw-previous.yaw),std::cos(next.yaw-previous.yaw));
    const double travel=std::hypot(next.x-previous.x,next.y-previous.y)+
        std::hypot(APA_VEHICLE_REAR_TO_FRONT_M+.20,APA_VEHICLE_WIDTH_M/2+.15)*std::fabs(dyaw);
    const int samples=std::max(1,int(std::ceil(travel/kSampleStep)));
    for(int i=1;i<=samples;++i) {
      const double fraction=double(i)/samples;
      const double progress=previous_distance+fraction*(origin_progress+distance-previous_distance);
      const Pose sample{previous.x+fraction*(next.x-previous.x),previous.y+fraction*(next.y-previous.y),previous.yaw+fraction*dyaw};
      result=check(sample,progress,&state,whole_path?"planning_clearance":"path_blocked");
      scene.Observe(observation,sample,progress,result,-2,mode);
      if(result.blocked()) { result.set_checked_distance_m(distance);return result; }
    }
    previous=next;previous_distance=origin_progress+distance;
    if(distance>=preview) break;

    while(vertex<vertices.size() && vertices[vertex]<=distance+1.e-9) ++vertex;
    double next_distance=std::min(preview,distance+.05);
    if(vertex<vertices.size()) next_distance=std::min(next_distance,vertices[vertex]);
    if(recovering) {
      const double join=recovery->info.prefix_length-origin_progress;
      if(join>distance+1.e-9) next_distance=std::min(next_distance,join);
    }
    distance=next_distance;
  }
  return result;
}

ApaOnlineSafety CheckApaOnlineSafety(const ApaInput& input,
    const ::apa::ApaParkingPathPlanningOutputData* path,
    const ApaMocoCtrlChartV2State_t& chart,double stop_distance,double preview_speed,
    ApaClearanceRecovery* recovery,const ApaMotionPrediction* motion,
    const ApaMotionPredictionPolicy* prediction_policy,
    ApaCandidateSafetyObservation* observation) {
  return ApaSafetyScene(input).CheckOnlineSafety(input,path,chart,stop_distance,preview_speed,
      recovery,motion,prediction_policy,observation);
}
ApaOnlineSafety ApaSafetyScene::CheckOnlineSafety(const ApaInput& input,
    const ::apa::ApaParkingPathPlanningOutputData* path,
    const ApaMocoCtrlChartV2State_t& chart,double stop_distance,double preview_speed,
    ApaClearanceRecovery* recovery,const ApaMotionPrediction* motion,
    const ApaMotionPredictionPolicy* prediction_policy,
    ApaCandidateSafetyObservation* observation) const {
  if (observation) *observation = {};
  return CheckPath(SafetyScene(input,*impl_),input,path,chart,stop_distance,preview_speed,false,recovery,motion,
      prediction_policy,recovery,observation);
}

static ApaOnlineSafety CheckCandidatePath(const ApaInput& input,
    const ::apa::ApaParkingPathPlanningOutputData& path, ApaCandidateSafetyObservation* observation,
    const ApaClearanceRecovery* recovery,
    Real goal_position_tolerance, Real goal_yaw_tolerance) {
  if (path.pathPointCount < 2 || path.pathPointCount > static_cast<int>(path.pathPointX.size())) {
    ApaOnlineSafety rejected;
    rejected.set_blocked(true); rejected.set_reason("invalid_candidate_count");
    return rejected;
  }
  ApaOnlineSafety rejected;
  rejected.set_scene_version(input.scene().version());
  rejected.set_blocked(true);
  if (!apaGoalToleranceValid(goal_position_tolerance) || !apaGoalToleranceValid(goal_yaw_tolerance)) {
    rejected.set_reason("invalid_goal_tolerance"); return rejected;
  }
  double position_tolerance = apaGoalPositionTolerance(goal_position_tolerance);
  double yaw_tolerance = apaGoalYawTolerance(goal_yaw_tolerance);
  if (input.has_goal_tolerance()) {
    position_tolerance = std::min(position_tolerance, input.goal_tolerance().position_m());
    yaw_tolerance = std::min(yaw_tolerance, input.goal_tolerance().yaw_rad());
  }
  const int end = path.pathPointCount - 1;
  double length = 0.0;
  for (int i = 0; i <= end; ++i) {
    if (!std::isfinite(path.pathPointX[i]) || !std::isfinite(path.pathPointY[i]) ||
        !std::isfinite(path.pathPointTheta[i]) || !std::isfinite(path.pathPointOdometer[i])) {
      rejected.set_reason("invalid_candidate_geometry"); return rejected;
    }
    if (i) length += std::hypot(path.pathPointX[i]-path.pathPointX[i-1],
        path.pathPointY[i]-path.pathPointY[i-1]);
  }
  if (!ValidRecovery(recovery)) {
    rejected.set_reason("invalid_clearance_recovery"); return rejected;
  }
  if (recovery && recovery->info.enabled) {
    const auto& info = recovery->info;
    const double start_yaw = path.pathPointTheta[0]-info.origin_yaw;
    if (std::hypot(path.pathPointX[0]-info.origin_x, path.pathPointY[0]-info.origin_y) > 1.e-4 ||
        std::fabs(std::atan2(std::sin(start_yaw),std::cos(start_yaw))) > 1.e-4 ||
        info.prefix_length > length+1.e-5) {
      rejected.set_reason("invalid_clearance_recovery"); return rejected;
    }

    Integer index = 0;
    Real x, y, yaw;
    apaMocoPathAtDistance(path.pathPointX.data(), path.pathPointY.data(),
        path.pathPointTheta.data(), path.pathPointCount, info.prefix_length,
        &index, &x, &y, &yaw);
    const auto join = SafetyScene(input).PlanningCheck({x,y,yaw},
        kApaObstaclePredictionSec,"clearance_not_restored");
    SafetyScene(input).Observe(observation,{x,y,yaw},info.prefix_length,join,-2);
    if (join.blocked()) return join;
  }
  const double distance = std::hypot(path.pathPointX[end] - input.target_x(),
      path.pathPointY[end] - input.target_y());
  const double yaw_error = std::atan2(std::sin(path.pathPointTheta[end] - input.target_theta()),
      std::cos(path.pathPointTheta[end] - input.target_theta()));
  if (!std::isfinite(distance) || distance > position_tolerance) {
    rejected.set_reason("candidate_goal_position");
    SafetyScene(input).Observe(observation,{path.pathPointX[end],path.pathPointY[end],path.pathPointTheta[end]},length,rejected,-2);
    return rejected;
  }
  if (!std::isfinite(yaw_error) || std::fabs(yaw_error) > yaw_tolerance) {
    rejected.set_reason("candidate_goal_heading");
    SafetyScene(input).Observe(observation,{path.pathPointX[end],path.pathPointY[end],path.pathPointTheta[end]},length,rejected,-2);
    return rejected;
  }
  ApaMocoCtrlChartV2State_t chart{};
  chart.StpNum = 1; chart.SegEndIdx[0] = path.pathPointCount-1;
  return CheckPath(SafetyScene(input),input, &path, chart, 0.0, 0.0, true, recovery,nullptr,nullptr,nullptr,observation);
}
ApaOnlineSafety CheckApaCandidatePathObserved(const ApaInput& input,
    const ::apa::ApaParkingPathPlanningOutputData& path, ApaCandidateSafetyObservation* observation,
    const ApaClearanceRecovery* recovery, Real position, Real yaw) {
  if (observation) *observation={};
  auto result=CheckCandidatePath(input,path,observation,recovery,position,yaw);
  if (observation && result.blocked() && observation->stage.empty()) observation->stage=result.reason();
  return result;
}

ApaOnlineSafety CheckApaCandidatePath(const ApaInput& input,
    const ::apa::ApaParkingPathPlanningOutputData& path,
    const ApaClearanceRecovery* recovery, Real position, Real yaw) {
  return CheckCandidatePath(input,path,nullptr,recovery,position,yaw);
}

}
