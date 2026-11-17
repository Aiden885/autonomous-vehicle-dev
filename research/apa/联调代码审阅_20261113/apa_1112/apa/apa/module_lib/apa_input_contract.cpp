#include "apa_input_contract.h"
#include "apa_scene_limits.h"

#include <nlohmann/json.hpp>
#include <cmath>
#include <cstdio>
#include <limits>
#include <stdexcept>
#include <vector>
#include "apaPlanningTypes.h"
#include "apaVehicleDefaults.h"

namespace pangu::modules {
namespace {
using Json = nlohmann::json;
constexpr const char* kEgo = "gaasd.carla.ego_state.v1";
constexpr const char* kRequest = "gaasd.carla.apa_plan_request.v1";
constexpr double kPi = 3.14159265358979323846;
struct BadInput : std::runtime_error { using std::runtime_error::runtime_error; };
struct Number {
  const char* name;
  const char* alias;
  double low, high;
};
const Number kPose[] = {{"vehicle_x", "VehicleX", -1e6, 1e6},
    {"vehicle_y", "VehicleY", -1e6, 1e6}, {"vehicle_yaw", "VehicleYaw", -2*kPi, 2*kPi}};
const Number kFeedback[] = {{"vehicle_speed", "VehicleSpeed", -100, 100},
    {"vehicle_motion_speed", "VehicleMotionSpeed", 0, 100},
    {"vehicle_acceleration", "VehicleAcceleration", -100, 100},
    {"road_gravity_x", "RoadGravityX", -9.80665, 9.80665},
    {"road_gravity_y", "RoadGravityY", -9.80665, 9.80665},
    {"vehicle_steering_command_rad", "VehicleSteeringCommandRad", -1.5, 1.5},
    {"cycle_time", "CycleTime", 1e-6, 0.02}};
const Number kGeometry[] = {{"target_x", "TarX", -1e6, 1e6},
    {"target_y", "TarY", -1e6, 1e6}, {"target_theta", "TarTheta", -2*kPi, 2*kPi},
    {"r_min", "Rmin", 1e-6, 1000}, {"entry_point_x", "EntrPtX", -1e6, 1e6},
    {"entry_point_y", "EntrPtY", -1e6, 1e6},
    {"slot_point1_x", "SltPt1X", -1e6, 1e6}, {"slot_point1_y", "SltPt1Y", -1e6, 1e6},
    {"slot_point2_x", "SltPt2X", -1e6, 1e6}, {"slot_point2_y", "SltPt2Y", -1e6, 1e6},
    {"slot_point3_x", "SltPt3X", -1e6, 1e6}, {"slot_point3_y", "SltPt3Y", -1e6, 1e6},
    {"slot_point4_x", "SltPt4X", -1e6, 1e6}, {"slot_point4_y", "SltPt4Y", -1e6, 1e6},
    {"angle_offset", "AgrOfst", -1000, 1000},
    {"shortcut_threshold", "ShortCutThd", 0, 1000}};
const Number kFeedbackEnums[] = {{"gear_pos", "GearPos", 0, 3},
    {"parking_action", "ParkingAction", 1, 2}};
const Number kRequestEnums[] = {{"parking_action", "ParkingAction", 1, 2},
    {"target_slot_kind", "TargetSlotKind", 1, 4}, {"slot_entry", "SltEntr", 1, 4},
    {"parking_type", "PrkType", 0, 3}, {"surrounding_type", "SurdType", 0, 4}};
struct Group {
  const char* count;
  const char* count_alias;
  int capacity;
  std::vector<std::pair<const char*, const char*>> fields;
};
const Group kGroups[] = {
  {"boundary_line_num", "BoundaryLineNum", APA_SCENARIO_BOUNDARY_LINE_MAX,
    {{"boundary_line_x1", "BoundaryLineX1"}, {"boundary_line_y1", "BoundaryLineY1"},
     {"boundary_line_x2", "BoundaryLineX2"}, {"boundary_line_y2", "BoundaryLineY2"}}},
  {"virtual_boundary_line_num", "VirtualBoundaryLineNum", APA_SCENARIO_BOUNDARY_LINE_MAX,
    {{"virtual_boundary_p1_x", "VirtualBoundaryP1X"}, {"virtual_boundary_p1_y", "VirtualBoundaryP1Y"},
     {"virtual_boundary_p2_x", "VirtualBoundaryP2X"}, {"virtual_boundary_p2_y", "VirtualBoundaryP2Y"}}},
  {"slot_track_stopper_num", "SlotTrackStopperNum", APA_SCENARIO_BOUNDARY_LINE_MAX,
    {{"slot_track_stopper_x1", "SlotTrackStopperX1"}, {"slot_track_stopper_y1", "SlotTrackStopperY1"},
     {"slot_track_stopper_x2", "SlotTrackStopperX2"}, {"slot_track_stopper_y2", "SlotTrackStopperY2"}}},
  {"boundary_sep_point_num", "BoundarySepPtNum", APA_SCENARIO_BOUNDARY_POINT_MAX,
    {{"boundary_sep_point_x", "BoundarySepPtX"}, {"boundary_sep_point_y", "BoundarySepPtY"}}},
  {"stopper_point_num", "StopperPtNum", APA_SCENARIO_BOUNDARY_POINT_MAX,
    {{"stopper_point_x", "StopperPtX"}, {"stopper_point_y", "StopperPtY"}}}
};

const Json& Required(const Json& object, const char* name, const char* alias = nullptr) {
  if (!object.is_object()) throw BadInput("expected_object");
  const auto first = object.find(name);
  const auto second = alias ? object.find(alias) : object.end();
  if (first == object.end() && second == object.end()) throw BadInput(std::string("missing:") + name);
  if (first != object.end() && second != object.end() && *first != *second)
    throw BadInput(std::string("alias_conflict:") + name);
  return first != object.end() ? *first : *second;
}

double Finite(const Json& value, const char* name) {
  if (!value.is_number()) throw BadInput(std::string("expected_number:") + name);
  const double result = value.get<double>();
  if (!std::isfinite(result)) throw BadInput(std::string("non_finite:") + name);
  return result;
}

uint64_t Unsigned(const Json& value, const char* name, bool nonzero = true) {
  if (!value.is_number_integer() || (!value.is_number_unsigned() && value.get<int64_t>() < 0))
    throw BadInput(std::string("expected_unsigned_integer:") + name);
  const uint64_t result = value.get<uint64_t>();
  if (nonzero && result == 0) throw BadInput(std::string("zero_identity:") + name);
  return result;
}

std::string String(const Json& value, const char* name) {
  if (!value.is_string()) throw BadInput(std::string("expected_string:") + name);
  return value.get<std::string>();
}

bool SessionValid(const std::string& session) {
  if (session.empty() || session.size() > 128) return false;
  for (unsigned char c : session)
    if (!((c >= 'a' && c <= 'z') || (c >= 'A' && c <= 'Z') ||
          (c >= '0' && c <= '9') || c == '-' || c == '_' || c == '.')) return false;
  return true;
}

const google::protobuf::FieldDescriptor* Field(const char* name) {
  const auto* field = ApaInput::descriptor()->FindFieldByName(name);
  if (field == nullptr) throw std::logic_error("unknown APA contract field");
  return field;
}

void ReadNumbers(const Json& payload, const Number* begin, const Number* end,
                 bool integers, ApaInput* out) {
  const auto* reflection = out->GetReflection();
  for (auto spec = begin; spec != end; ++spec) {
    const auto& value = Required(payload, spec->name, spec->alias);
    if (integers) {
      Unsigned(value, spec->name, false);
      if (payload.contains(spec->name) && payload.contains(spec->alias))
        Unsigned(payload.at(spec->alias), spec->alias, false);
    } else if (payload.contains(spec->name) && payload.contains(spec->alias)) {
      Finite(payload.at(spec->alias), spec->alias);
    }
    const double number = Finite(value, spec->name);
    if (number < spec->low || number > spec->high)
      throw BadInput(std::string("out_of_range:") + spec->name);
    if (integers) reflection->SetInt32(out, Field(spec->name), static_cast<int32_t>(number));
    else reflection->SetDouble(out, Field(spec->name), number);
  }
}

template <size_t N>
void ReadNumbers(const Json& payload, const Number (&specs)[N], bool integers, ApaInput* out) {
  ReadNumbers(payload, specs, specs + N, integers, out);
}

void ReadBool(const Json& payload, const char* name, const char* alias, ApaInput* out) {
  const auto& value = Required(payload, name, alias);
  if (!value.is_boolean() || (alias && payload.contains(alias) && !payload.at(alias).is_boolean()))
    throw BadInput(std::string("expected_boolean:") + name);
  out->GetReflection()->SetBool(out, Field(name), value.get<bool>());
}

bool NumbersValid(const ApaInput& in, const Number* begin, const Number* end, bool integers,
                  std::string* error) {
  const auto* r = in.GetReflection();
  for (auto spec = begin; spec != end; ++spec) {
    const double value = integers ? r->GetInt32(in, Field(spec->name)) : r->GetDouble(in, Field(spec->name));
    if (!std::isfinite(value) || value < spec->low || value > spec->high) {
      *error = std::string("out_of_range:") + spec->name;
      return false;
    }
  }
  return true;
}

template <size_t N>
bool NumbersValid(const ApaInput& in, const Number (&specs)[N], bool integers, std::string* error) {
  return NumbersValid(in, specs, specs + N, integers, error);
}

bool SafetyValid(const ApaSafetyState& safety) {
  if (!std::isfinite(safety.standstill_sec()) || safety.standstill_sec() < 0.0 || safety.reason().size() > 512)
    return false;
  if (safety.state() == "none") return safety.reason().empty() && safety.standstill_sec() == 0.0;
  return safety.event_id() != 0 && !safety.reason().empty() &&
      (safety.state() == "braking" || safety.state() == "stopped" || safety.state() == "unconfirmed");
}

void ReadMeta(const Json& message, const std::string& topic, ApaInput* out) {
  const auto& header = Required(message, "header");
  const auto& payload = Required(message, "payload");
  if (String(Required(header, "protocol"), "protocol") != "gaasd_carla_bridge" ||
      String(Required(header, "protocol_version"), "protocol_version") != kApaInputProtocolVersion ||
      String(Required(header, "message_type"), "message_type") != topic)
    throw BadInput("protocol_mismatch");
  if (String(Required(header, "coordinate_frame"), "coordinate_frame") != "carla_world" ||
      String(Required(payload, "vehicle_reference_point"), "vehicle_reference_point") != "rear_axle_center")
    throw BadInput("coordinate_or_reference_mismatch");
  const auto session = String(Required(payload, "session_id"), "session_id");
  const auto profile = String(Required(payload, "vehicle_profile_id"), "vehicle_profile_id");
  if (profile != APA_VEHICLE_PROFILE_ID) throw BadInput("vehicle_profile_mismatch");
  if (!SessionValid(session)) throw BadInput("invalid_session_id");
  const auto task = Unsigned(Required(payload, "task_id"), "task_id");
  const auto frame = Unsigned(Required(payload, "carla_frame_id"), "carla_frame_id");
  const double time = Finite(Required(payload, "sim_time_sec"), "sim_time_sec");
  const double dt = Finite(Required(header, "delta_time_sec"), "delta_time_sec");
  if (time < 0.0 || dt < 1e-6 || dt > 0.02 ||
      std::fabs(0.02 / dt - std::round(0.02 / dt)) > 1e-4 ||
      frame != Unsigned(Required(header, "frame_id"), "header.frame_id") ||
      frame != Unsigned(Required(payload, "frame_id"), "frame_id") ||
      std::fabs(time - Finite(Required(header, "sim_time_sec"), "header.sim_time_sec")) > 1e-6)
    throw BadInput("invalid_or_inconsistent_clock");
  out->set_session_id(session);
  out->set_vehicle_profile_id(profile);
  out->set_task_id(task);
  out->set_frame_id(static_cast<uint32_t>(frame));
  out->set_carla_frame_id(frame);
  out->set_sim_time_sec(time);
  out->set_cycle_time(dt);
}

void ReadSafety(const Json& value, ApaSafetyState* safety) {
  safety->set_event_id(Unsigned(Required(value, "event_id"), "safety.event_id", false));
  safety->set_reason(String(Required(value, "reason"), "safety.reason"));
  safety->set_state(String(Required(value, "state"), "safety.state"));
  safety->set_standstill_sec(Finite(Required(value, "standstill_sec"), "safety.standstill_sec"));
}

void ReadFeedback(const Json& payload, ApaInput* out) {
  const double header_dt = out->cycle_time();
  ReadNumbers(payload, kPose, false, out);
  ReadNumbers(payload, kFeedback, false, out);
  ReadNumbers(payload, kFeedbackEnums, true, out);
  ReadBool(payload, "parking_enabled", nullptr, out);
  if (payload.contains("planning_only")) ReadBool(payload, "planning_only", nullptr, out);
  if (std::fabs(header_dt - out->cycle_time()) > 1e-6) throw BadInput("cycle_time_mismatch");
  ReadSafety(Required(payload, "safety_state"), out->mutable_safety_state());
  const auto& recovery = Required(payload, "recovery");
  out->mutable_recovery()->set_recovery_id(Unsigned(Required(recovery, "recovery_id"), "recovery_id", false));
  ReadSafety(Required(recovery, "stop"), out->mutable_recovery()->mutable_stop());
  const auto& scene = Required(payload, "scene");
  auto* target = out->mutable_scene();
  target->set_version(Unsigned(Required(scene, "version"), "scene.version"));
  const auto& obstacles = Required(scene, "obstacles");
  if (!obstacles.is_array() || obstacles.size() > kApaSceneMaxObstacles) throw BadInput("scene_obstacle_capacity");
  for (const auto& obstacle : obstacles) {
    auto* box = target->add_obstacles();
    box->set_id(String(Required(obstacle, "id"), "obstacle.id"));
    box->set_velocity_x(Finite(Required(obstacle, "velocity_x"), "obstacle.velocity_x"));
    box->set_velocity_y(Finite(Required(obstacle, "velocity_y"), "obstacle.velocity_y"));
    const auto& points = Required(obstacle, "footprint");
    if (!points.is_array() || points.size() < 3 || points.size() > 8) throw BadInput("obstacle_vertex_count");
    for (const auto& point : points) {
      auto* xy = box->add_footprint();
      xy->set_x(Finite(Required(point, "x"), "footprint.x"));
      xy->set_y(Finite(Required(point, "y"), "footprint.y"));
    }
  }
  std::string error;
  if (!ValidateApaFeedback(*out, &error)) throw BadInput(error);
}

void ReadRequest(const Json& payload, ApaInput* out) {
  ReadNumbers(payload, kPose, false, out);
  ReadNumbers(payload, kGeometry, false, out);
  ReadNumbers(payload, kRequestEnums, true, out);
  ReadBool(payload, "shortcut_switch", "ShortCutSw", out);
  ReadBool(payload, "need_hybrid_algorithm_fusion", "NeedHybridAlgFusion", out);
  const auto* reflection = out->GetReflection();
  for (const auto& group : kGroups) {
    const Number count{group.count, group.count_alias, 0, static_cast<double>(group.capacity)};
    ReadNumbers(payload, &count, &count + 1, true, out);
    const int expected = reflection->GetInt32(*out, Field(group.count));
    for (const auto& names : group.fields) {
      const auto& values = Required(payload, names.first, names.second);
      if (!values.is_array() || values.size() != static_cast<size_t>(expected))
        throw BadInput(std::string("array_length:") + names.first);
      for (const auto& value : values) {
        const double number = Finite(value, names.first);
        if (std::fabs(number) > 1e6) throw BadInput(std::string("out_of_range:") + names.first);
        reflection->AddDouble(out, Field(names.first), number);
      }
    }
  }
  if (payload.contains("goal_tolerance")) {
    const auto& value = payload.at("goal_tolerance");
    auto* goal = out->mutable_goal_tolerance();
    goal->set_position_m(Finite(Required(value, "position_m"), "goal_tolerance.position_m"));
    goal->set_yaw_rad(Finite(Required(value, "yaw_rad"), "goal_tolerance.yaw_rad"));
  }
  std::string error;
  if (!ValidateApaRequest(*out, &error)) throw BadInput(error);
}

ApaInput Geometry(ApaInput input) {
  input.clear_frame_id(); input.clear_carla_frame_id(); input.clear_sim_time_sec();
  input.clear_cycle_time(); input.clear_session_id(); input.clear_task_id();
  input.clear_vehicle_x(); input.clear_vehicle_y(); input.clear_vehicle_yaw();
  input.clear_vehicle_speed(); input.clear_vehicle_acceleration(); input.clear_gear_pos();
  input.clear_vehicle_motion_speed();
  input.clear_road_gravity_x(); input.clear_road_gravity_y(); input.clear_vehicle_steering_command_rad();
  input.clear_planning_only();
  input.clear_parking_enabled(); input.clear_safety_state(); input.clear_input_validation(); input.clear_scene(); input.clear_recovery();
  return input;
}

void CopyFeedback(const ApaInput& source, ApaInput* out) {
  out->set_vehicle_profile_id(source.vehicle_profile_id());
  out->set_frame_id(source.frame_id()); out->set_carla_frame_id(source.carla_frame_id());
  out->set_session_id(source.session_id()); out->set_task_id(source.task_id());
  out->set_sim_time_sec(source.sim_time_sec()); out->set_cycle_time(source.cycle_time());
  out->set_vehicle_x(source.vehicle_x()); out->set_vehicle_y(source.vehicle_y());
  out->set_vehicle_yaw(source.vehicle_yaw()); out->set_vehicle_speed(source.vehicle_speed());
  out->set_vehicle_acceleration(source.vehicle_acceleration()); out->set_gear_pos(source.gear_pos());
  out->set_vehicle_motion_speed(source.vehicle_motion_speed());
  out->set_road_gravity_x(source.road_gravity_x()); out->set_road_gravity_y(source.road_gravity_y());
  out->set_vehicle_steering_command_rad(source.vehicle_steering_command_rad());
  out->set_parking_enabled(source.parking_enabled()); out->set_parking_action(source.parking_action());
  out->set_planning_only(source.planning_only());
  *out->mutable_safety_state() = source.safety_state();
  *out->mutable_scene() = source.scene();
  *out->mutable_recovery() = source.recovery();
}

bool RequestClockValid(const ApaInput& request, const ApaInput& feedback) {
  const double delta = request.sim_time_sec() - feedback.sim_time_sec();
  const double frames = request.carla_frame_id() >= feedback.carla_frame_id()
      ? static_cast<double>(request.carla_frame_id() - feedback.carla_frame_id())
      : -static_cast<double>(feedback.carla_frame_id() - request.carla_frame_id());
  return std::fabs(request.cycle_time() - feedback.cycle_time()) < 1e-6 &&
      delta >= -0.5 && delta <= feedback.cycle_time() + 1e-5 &&
      std::fabs(delta - frames * feedback.cycle_time()) < 1e-5;
}

}

bool ValidateApaFeedback(const ApaInput& in, std::string* error) {
  if (in.vehicle_profile_id() != APA_VEHICLE_PROFILE_ID) {
    *error = "vehicle_profile_mismatch"; return false;
  }
  if (!SessionValid(in.session_id()) || in.task_id() == 0 || in.carla_frame_id() == 0 ||
      !std::isfinite(in.sim_time_sec()) || in.sim_time_sec() < 0.0) {
    *error = "invalid_feedback_identity"; return false;
  }
  if (!NumbersValid(in, kPose, false, error) || !NumbersValid(in, kFeedback, false, error) ||
      !NumbersValid(in, kFeedbackEnums, true, error)) return false;
  if (in.vehicle_motion_speed()+1.e-4 < std::fabs(in.vehicle_speed())) {
    *error="invalid_motion_speed"; return false;
  }
  if (in.planning_only() && (!in.parking_enabled() || in.parking_action() != 1)) {
    *error = "invalid_planning_only_mode"; return false;
  }
  const double ratio = 0.02 / in.cycle_time();
  if (std::fabs(ratio - std::round(ratio)) > 1e-4) { *error = "invalid_cycle_time"; return false; }
  if (!in.has_safety_state() || !SafetyValid(in.safety_state())) { *error = "invalid_safety_state"; return false; }
  if (!in.has_recovery() || !in.recovery().has_stop() || !SafetyValid(in.recovery().stop()) ||
      (!in.recovery().stop().reason().empty() && in.recovery().recovery_id() == 0)) {
    *error = "invalid_recovery_feedback"; return false;
  }
  if (!in.has_scene() || in.scene().version() != in.carla_frame_id() ||
      in.scene().obstacles_size() > kApaSceneMaxObstacles) {
    *error = "invalid_scene_version_or_capacity"; return false;
  }
  for (const auto& obstacle : in.scene().obstacles()) {
    const int count = obstacle.footprint_size();
    if (obstacle.id().empty() || obstacle.id().size() > 96 || count < 3 || count > 8 ||
        !std::isfinite(obstacle.velocity_x()) || std::fabs(obstacle.velocity_x()) > 100 ||
        !std::isfinite(obstacle.velocity_y()) || std::fabs(obstacle.velocity_y()) > 100) {
      *error = "invalid_scene_obstacle"; return false;
    }
    double orientation = 0.0;
    for (int i = 0; i < count; ++i) {
      const auto& a = obstacle.footprint(i);
      const auto& b = obstacle.footprint((i+1) % count);
      if (!std::isfinite(a.x()) || !std::isfinite(a.y()) || std::fabs(a.x()) > 1e6 || std::fabs(a.y()) > 1e6) {
        *error = "invalid_scene_coordinate"; return false;
      }
      for (int j = 0; j < count; ++j) {
        if (j == i || j == (i+1) % count) continue;
        const auto& c = obstacle.footprint(j);
        const double cross = (b.x()-a.x())*(c.y()-a.y()) - (b.y()-a.y())*(c.x()-a.x());
        if (!std::isfinite(cross) || std::fabs(cross) < 1e-8 || (orientation && cross*orientation <= 0.0)) {
          *error = "invalid_scene_convex_polygon"; return false;
        }
        orientation = cross;
      }
    }
  }
  return true;
}

bool ValidateApaRequest(const ApaInput& in, std::string* error) {
  if (in.vehicle_profile_id() != APA_VEHICLE_PROFILE_ID) {
    *error = "vehicle_profile_mismatch"; return false;
  }
  if (!NumbersValid(in, kGeometry, false, error) || !NumbersValid(in, kRequestEnums, true, error)) return false;
  const auto* r = in.GetReflection();
  for (const auto& group : kGroups) {
    const int count = r->GetInt32(in, Field(group.count));
    if (count < 0 || count > group.capacity) { *error = std::string("capacity:") + group.count; return false; }
    for (const auto& names : group.fields) {
      const auto* field = Field(names.first);
      if (r->FieldSize(in, field) != count) { *error = std::string("array_length:") + names.first; return false; }
      for (int i = 0; i < count; ++i) {
        const double value = r->GetRepeatedDouble(in, field, i);
        if (!std::isfinite(value) || std::fabs(value) > 1e6) {
          *error = std::string("out_of_range:") + names.first; return false;
        }
      }
    }
  }
  const double x[] = {in.slot_point1_x(), in.slot_point2_x(), in.slot_point3_x(), in.slot_point4_x()};
  const double y[] = {in.slot_point1_y(), in.slot_point2_y(), in.slot_point3_y(), in.slot_point4_y()};
  double orientation = 0.0;
  for (int i = 0; i < 4; ++i) {
    const int j = (i + 1) % 4, k = (i + 2) % 4;
    const double cross = (x[j]-x[i])*(y[k]-y[j]) - (y[j]-y[i])*(x[k]-x[j]);
    if (std::fabs(cross) < 1e-6 || (i && cross * orientation <= 0.0)) {
      *error = "invalid_slot_quadrilateral"; return false;
    }
    orientation = cross;
  }
  if (in.has_goal_tolerance()) {
    const auto& goal = in.goal_tolerance();
    if (!std::isfinite(goal.position_m()) || goal.position_m() <= 0.0 || goal.position_m() > 1000 ||
        !std::isfinite(goal.yaw_rad()) || goal.yaw_rad() <= 0.0 || goal.yaw_rad() > kPi) {
      *error = "invalid_goal_tolerance"; return false;
    }
  }
  return true;
}

std::string ApaRequestFingerprint(const ApaInput& input) { return Geometry(input).SerializeAsString(); }

bool ApaInputContract::IsOld(const Task& task) const {
  return retired_sessions_.count(task.session) || (have_feedback_ && task.session == feedback_.session_id() &&
      task.id < feedback_.task_id());
}

ApaInputContract::Request& ApaInputContract::RequestFor(const Task& task) {
  if (have_feedback_ && task == Task{feedback_.session_id(), feedback_.task_id()}) return current_request_;
  auto found = pending_requests_.find(task);
  if (found != pending_requests_.end()) return found->second;
  if (pending_requests_.size() >= 16) throw BadInput("pending_task_limit");
  auto& request = pending_requests_[task];
  request.task = task;
  return request;
}

void ApaInputContract::Compose(bool feedback_valid) {
  notice_ = (current_request_.valid && fault_.empty()) ? Geometry(current_request_.input) : ApaInput{};
  CopyFeedback(feedback_, &notice_);
  auto* status = notice_.mutable_input_validation();
  status->set_feedback_valid(feedback_valid);
  status->set_request_valid(current_request_.valid && fault_.empty());
  status->set_error(fault_);
}

bool ApaInputContract::Reject(const Task& task, const std::string& reason) {
  if (task.id == 0 && !task.session.empty()) return Reject({}, reason);
  if (!task.session.empty() && IsOld(task)) return false;
  if (have_feedback_ && (task.session.empty() || task == Task{feedback_.session_id(), feedback_.task_id()})) {
    if (fault_.empty()) {
      fault_ = reason;
      std::fprintf(stderr, "[APA input] rejected task=%llu reason=%s\n",
          static_cast<unsigned long long>(feedback_.task_id()), reason.c_str());
    }

    Compose(false);
    return true;
  }
  if (!task.session.empty()) {
    try {
      auto& request = RequestFor(task);
      if (request.error.empty()) request.error = reason;
    } catch (const BadInput&) {
      return Reject({}, "pending_task_limit");
    }
  } else if (unbound_error_.empty()) unbound_error_ = reason;
  return false;
}

bool ApaInputContract::RejectMessage(const std::string& topic, const std::string& reason) {
  if (topic != kEgo && topic != kRequest) return false;
  return Reject({}, reason);
}

bool ApaInputContract::Consume(const std::string& topic, const std::string& text) {
  if (topic != kEgo && topic != kRequest) return false;
  Task task;
  ApaInput parsed;
  try {
    if (text.size() > kApaSceneMaxMessageBytes) throw BadInput("message_too_large");
    std::vector<std::set<std::string>> keys;
    const Json message = Json::parse(text, [&keys](int depth, Json::parse_event_t event, Json& value) {
      if (depth > 32) throw BadInput("json_nesting_limit");
      if (event == Json::parse_event_t::object_start) keys.emplace_back();
      else if (event == Json::parse_event_t::key && !keys.back().insert(value.get<std::string>()).second)
        throw BadInput("duplicate_json_key");
      else if (event == Json::parse_event_t::object_end) keys.pop_back();
      return true;
    });
    const auto& payload = Required(message, "payload");
    task.session = String(Required(payload, "session_id"), "session_id");
    if (!SessionValid(task.session)) { task.session.clear(); throw BadInput("invalid_session_id"); }
    task.id = Unsigned(Required(payload, "task_id"), "task_id");
    if (IsOld(task)) return false;
    ReadMeta(message, topic, &parsed);
    if (topic == kRequest) {
      const auto pending = pending_requests_.find(task);
      const Request* existing = have_feedback_ && task == Task{feedback_.session_id(), feedback_.task_id()}
          ? &current_request_ : (pending != pending_requests_.end() ? &pending->second : nullptr);
      if (existing && existing->valid &&
          parsed.carla_frame_id() < existing->input.carla_frame_id()) return false;
      ReadRequest(payload, &parsed);
      if (have_feedback_ && task == Task{feedback_.session_id(), feedback_.task_id()} &&
          (parsed.parking_action() != feedback_.parking_action() ||
           !RequestClockValid(parsed, feedback_)))
        throw BadInput("request_action_or_clock_mismatch");
      if (existing && existing->valid &&
          ApaRequestFingerprint(existing->input) != ApaRequestFingerprint(parsed))
        throw BadInput("request_changed_without_new_task");
      auto& request = RequestFor(task);
      request.task = task;
      request.input = parsed;
      request.valid = true;
      if (have_feedback_ && task == Task{feedback_.session_id(), feedback_.task_id()}) {
        if (!request.error.empty() && fault_.empty()) fault_ = request.error;
        Compose();
        return true;
      }
      return false;
    }
    if (have_feedback_ && task == Task{feedback_.session_id(), feedback_.task_id()} &&
        parsed.carla_frame_id() < feedback_.carla_frame_id()) return false;
    ReadFeedback(payload, &parsed);
    const bool same_task = have_feedback_ && task == Task{feedback_.session_id(), feedback_.task_id()};
    if (same_task && (parsed.parking_action() != feedback_.parking_action() ||
        (feedback_.parking_enabled() && parsed.planning_only() != feedback_.planning_only()) ||
        (feedback_.parking_enabled() && !parsed.parking_enabled())))
      throw BadInput("task_state_changed_without_new_task");
    if (same_task && parsed.recovery().recovery_id() < feedback_.recovery().recovery_id())
      throw BadInput("recovery_id_regressed");
    if (same_task && parsed.carla_frame_id() == feedback_.carla_frame_id()) {
      auto previous = feedback_, candidate = parsed;
      previous.clear_safety_state(); candidate.clear_safety_state();
      previous.clear_recovery(); candidate.clear_recovery();
      if (previous.SerializeAsString() != candidate.SerializeAsString())
        throw BadInput("feedback_changed_within_frame");
    } else if (same_task) {
      const auto frames = parsed.carla_frame_id() - feedback_.carla_frame_id();
      const double elapsed = parsed.sim_time_sec() - feedback_.sim_time_sec();

      if (elapsed <= 0.0 || std::fabs(parsed.cycle_time() - feedback_.cycle_time()) > 1e-6 ||
          std::fabs(elapsed - static_cast<double>(frames) * parsed.cycle_time()) > 1e-5 ||
          (parsed.parking_enabled() && frames != 1)) throw BadInput("feedback_clock_discontinuity");
    }
    if (!same_task) {
      if (have_feedback_ && task.session != feedback_.session_id()) {
        if (retired_sessions_.size() >= 256) throw BadInput("too_many_retired_sessions");
        retired_sessions_.insert(feedback_.session_id());
      }
      const auto pending = pending_requests_.find(task);
      current_request_ = pending != pending_requests_.end() ? pending->second : Request{};
      current_request_.task = task;
      for (auto it = pending_requests_.begin(); it != pending_requests_.end();) {
        if (retired_sessions_.count(it->first.session) ||
            (it->first.session == task.session && it->first.id <= task.id))
          it = pending_requests_.erase(it);
        else ++it;
      }
      fault_ = current_request_.error.empty() ? unbound_error_ : current_request_.error;
      unbound_error_.clear();
    } else if (!feedback_.safety_state().reason().empty() &&
        (parsed.safety_state().reason().empty() || parsed.safety_state().event_id() < feedback_.safety_state().event_id())) {
      *parsed.mutable_safety_state() = feedback_.safety_state();
    }
    feedback_ = parsed;
    have_feedback_ = true;
    if (current_request_.valid && feedback_.parking_action() != current_request_.input.parking_action())
      return Reject(task, "request_action_mismatch");
    if (!same_task && current_request_.valid && !RequestClockValid(current_request_.input, feedback_))
      return Reject(task, "request_clock_mismatch");
    Compose();
    return true;
  } catch (const BadInput& error) {
    return Reject(task, error.what());
  } catch (const Json::exception&) {
    return Reject(task, "malformed_json");
  }
}

}
