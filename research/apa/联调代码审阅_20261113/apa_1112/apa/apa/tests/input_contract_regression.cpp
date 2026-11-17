#include "apa_input_contract.h"
#include "apa_control_limits.h"
#include "apaVehicleDefaults.h"
#include <nlohmann/json.hpp>
#include <cassert>
#include <cmath>
#include <limits>
#include <string>

using nlohmann::json;
using pangu::modules::ApaInputContract;
constexpr const char* ego_topic = "gaasd.carla.ego_state.v1";
constexpr const char* request_topic = "gaasd.carla.apa_plan_request.v1";
void CheckOnlineSafetyCases();

json Envelope(const char* topic, int task = 1, int frame = 10, double time = 1.0,
              const char* session = "session-a") {
  json p{{"session_id", session}, {"task_id", task}, {"frame_id", frame},
      {"vehicle_profile_id", APA_VEHICLE_PROFILE_ID},
      {"carla_frame_id", frame}, {"sim_time_sec", time}, {"vehicle_reference_point", "rear_axle_center"},
      {"vehicle_x", 0.0}, {"vehicle_y", 0.0}, {"vehicle_yaw", 0.0}, {"parking_action", 1}};
  if (std::string(topic) == ego_topic) {
    p.update({{"cycle_time", 0.02}, {"vehicle_speed", 0.0}, {"vehicle_acceleration", 0.0},
        {"vehicle_motion_speed", 0.0}, {"road_gravity_x", 0.0}, {"road_gravity_y", 0.0},
        {"vehicle_steering_command_rad", 0.0},
        {"gear_pos", 1}, {"parking_enabled", true},
        {"scene", {{"version", frame}, {"obstacles", json::array()}}},
        {"recovery", {{"recovery_id", 0}, {"stop", {{"event_id", 0}, {"state", "none"}, {"reason", ""}, {"standstill_sec", 0.0}}}}},
        {"safety_state", {{"event_id", 0}, {"state", "none"}, {"reason", ""}, {"standstill_sec", 0.0}}}});
  } else {
    p.update({{"target_slot_kind", 1}, {"target_x", 5.0}, {"target_y", 1.0}, {"target_theta", 0.0},
        {"r_min", 5.5}, {"entry_point_x", 2.0}, {"entry_point_y", 1.0},
        {"slot_point1_x", 2.0}, {"slot_point1_y", 0.0}, {"slot_point2_x", 2.0}, {"slot_point2_y", 2.0},
        {"slot_point3_x", 6.0}, {"slot_point3_y", 2.0}, {"slot_point4_x", 6.0}, {"slot_point4_y", 0.0},
        {"slot_entry", 1}, {"parking_type", 2}, {"surrounding_type", 3}, {"angle_offset", 0.0},
        {"shortcut_switch", false}, {"shortcut_threshold", 0.0}, {"need_hybrid_algorithm_fusion", true},
        {"boundary_line_num", 0}, {"boundary_line_x1", json::array()}, {"boundary_line_y1", json::array()},
        {"boundary_line_x2", json::array()}, {"boundary_line_y2", json::array()},
        {"virtual_boundary_line_num", 0}, {"virtual_boundary_p1_x", json::array()},
        {"virtual_boundary_p1_y", json::array()}, {"virtual_boundary_p2_x", json::array()},
        {"virtual_boundary_p2_y", json::array()}, {"slot_track_stopper_num", 0},
        {"slot_track_stopper_x1", json::array()}, {"slot_track_stopper_y1", json::array()},
        {"slot_track_stopper_x2", json::array()}, {"slot_track_stopper_y2", json::array()},
        {"boundary_sep_point_num", 0}, {"boundary_sep_point_x", json::array()}, {"boundary_sep_point_y", json::array()},
        {"stopper_point_num", 0}, {"stopper_point_x", json::array()}, {"stopper_point_y", json::array()}});
  }
  return {{"header", {{"protocol", "gaasd_carla_bridge"},
      {"protocol_version", pangu::modules::kApaInputProtocolVersion}, {"message_type", topic},
      {"coordinate_frame", "carla_world"}, {"frame_id", frame}, {"sim_time_sec", time}, {"delta_time_sec", 0.02}}},
      {"payload", p}};
}

ApaInputContract Ready() {
  ApaInputContract store;
  assert(store.Consume(ego_topic, Envelope(ego_topic).dump()));
  assert(!store.GetInput().input_validation().request_valid());
  assert(store.Consume(request_topic, Envelope(request_topic).dump()));
  assert(store.GetInput().input_validation().request_valid());
  return store;
}

void InvalidRequest(const json& message) {
  auto store = Ready();
  assert(store.Consume(request_topic, message.dump()));
  const auto rejected = store.GetInput();
  assert(!rejected.input_validation().error().empty());
  assert(!rejected.input_validation().request_valid());
  assert(rejected.target_slot_kind() == 0);
  assert(rejected.vehicle_x() == 0.0);
  store.Consume(request_topic, Envelope(request_topic).dump());
  assert(!store.GetInput().input_validation().error().empty());
  store.Consume(request_topic, Envelope(request_topic, 2, 11, 1.02).dump());
  store.Consume(ego_topic, Envelope(ego_topic, 2, 11, 1.02).dump());
  assert(store.GetInput().input_validation().error().empty());
  assert(store.GetInput().input_validation().request_valid());
}

int main() {
  CheckOnlineSafetyCases();
  {
    auto mismatched = Envelope(ego_topic);
    mismatched["payload"]["vehicle_profile_id"] = "outdated-vehicle-profile";
    auto store = Ready();
    assert(store.Consume(ego_topic, mismatched.dump()));
    assert(store.GetInput().input_validation().error() == "vehicle_profile_mismatch");
  }
  {
    pangu::modules::ApaParams params;
    params.set_target_speed_mps(0.3);
    params.set_max_accel_mps2(0.2);
    params.set_max_steer_rad(0.2);
    ApaMocoParkingControlParam_t control{};
    apaMocoParkingControlDefaultParam(&control);
    std::string error;
    assert(pangu::modules::ApplyApaActuationLimits(params, &control, &error));
    ApaMocoParkingControlInput_t input{};
    input.Chart.PtCount = 2;
    input.Chart.PthPtX[1] = input.Chart.PthPtOdo[1] = 5.0f;
    input.Chart.PathUpdateIdx = 1;
    input.Chart.GearPos = 1;
    input.Chart.TargetSlotKind = input.Chart.ParkingAction = 1;
    ApaMocoParkingControlState_t state{};
    ApaMocoParkingControlOutput_t output{};
    for (int i = 0; i < 40; ++i) {
      apaMocoParkingControl(&input, &control, &state, &output);
      assert(std::fabs(output.Longi.TargetSpeed) <= 0.300001f);
      assert(std::fabs(output.Longi.TargetAcceleration) <= 0.200001f);
      assert(std::fabs(output.Lat.SteerAngle) <= 0.200001f);
    }
    assert(output.Longi.ControlEnabled && output.Longi.TargetAcceleration > 0.0f);
    assert(control.Longi.StopDeceleration <= 0.200001f);
    params.set_max_steer_rad(1.0);
    assert(!pangu::modules::ApplyApaActuationLimits(params, &control, &error));
    params.set_max_steer_rad(0.2);
    params.set_max_accel_mps2(std::numeric_limits<double>::quiet_NaN());
    assert(!pangu::modules::ApplyApaActuationLimits(params, &control, &error));
  }
  {
    pangu::modules::ApaParams params;
    params.set_target_speed_mps(1.1);
    params.set_max_accel_mps2(1.5);
    params.set_max_steer_rad(APA_VEHICLE_MAX_STEER_RAD);
    auto* limits = params.mutable_execution_limits();
    limits->set_cusp_position_m(0.2);
    limits->set_cusp_yaw_rad(0.1);
    limits->set_goal_position_m(0.1);
    limits->set_goal_yaw_rad(0.03);
    limits->set_standstill_speed_mps(0.02);
    limits->set_cusp_dwell_sec(0.2);
    limits->set_goal_dwell_sec(0.5);
    limits->set_gear_dwell_sec(0.2);
    limits->set_reference_distance_m(0.5);
    limits->set_reference_yaw_rad(0.6);
    ApaMocoParkingControlParam_t control{};
    std::string error;
    assert(pangu::modules::BuildApaControlCalibration(params, &control, &error));
    assert(control.Chart.StopDwellCycles == 10 && control.Chart.GoalDwellCycles == 25);
    limits->set_cusp_dwell_sec(1.e12);
    assert(!pangu::modules::BuildApaControlCalibration(params, &control, &error));
    limits->set_cusp_dwell_sec(0.2);
    limits->set_reference_distance_m(std::numeric_limits<double>::max());
    assert(!pangu::modules::BuildApaControlCalibration(params, &control, &error));
    limits->set_reference_distance_m(0.5);
    params.set_forward_ref_speed_mps(std::numeric_limits<double>::quiet_NaN());
    assert(!pangu::modules::BuildApaControlCalibration(params, &control, &error));
    params.set_forward_ref_speed_mps(std::numeric_limits<double>::max());
    assert(!pangu::modules::BuildApaControlCalibration(params, &control, &error));
    params.set_forward_ref_speed_mps(std::numeric_limits<double>::min());
    assert(!pangu::modules::BuildApaControlCalibration(params, &control, &error));
  }
  {
    auto store = Ready();
    auto feedback = Envelope(ego_topic, 1, 11, 1.02);
    feedback["payload"]["recovery"] = {{"recovery_id", 1},
        {"stop", {{"event_id", 1}, {"state", "braking"}, {"reason", "replan"}, {"standstill_sec", 0.0}}}};
    store.Consume(ego_topic, feedback.dump());
    feedback["payload"]["recovery"]["stop"]["state"] = "stopped";
    feedback["payload"]["recovery"]["stop"]["standstill_sec"] = 0.2;
    store.Consume(ego_topic, feedback.dump());
    assert(store.GetInput().input_validation().error().empty());
    assert(store.GetInput().recovery().stop().state() == "stopped");
    store.Consume(ego_topic, Envelope(ego_topic, 1, 12, 1.04).dump());
    assert(store.GetInput().input_validation().error() == "recovery_id_regressed");
  }
  {
    auto store = Ready();
    auto feedback = Envelope(ego_topic, 1, 11, 1.02);
    feedback["payload"]["scene"]["version"] = 10;
    store.Consume(ego_topic, feedback.dump());
    assert(store.GetInput().input_validation().error() == "invalid_scene_version_or_capacity");
  }
  {
    auto store = Ready();
    const auto fingerprint = pangu::modules::ApaRequestFingerprint(store.GetInput());
    auto feedback = Envelope(ego_topic, 1, 11, 1.02);
    json obstacle{{"id", "actor:12"}, {"velocity_x", 0.0}, {"velocity_y", 0.0},
        {"footprint", {{{"x", 5.0}, {"y", 0.0}}, {{"x", 6.0}, {"y", 0.0}},
                       {{"x", 6.0}, {"y", 1.0}}, {{"x", 5.0}, {"y", 1.0}}}}};
    feedback["payload"]["scene"]["obstacles"] = {obstacle};
    store.Consume(ego_topic, feedback.dump());
    assert(store.GetInput().input_validation().error().empty());
    assert(store.GetInput().scene().version() == 11 && store.GetInput().scene().obstacles_size() == 1);
    assert(pangu::modules::ApaRequestFingerprint(store.GetInput()) == fingerprint);
    feedback = Envelope(ego_topic, 1, 12, 1.04);
    obstacle["footprint"][1]["x"] = "bad";
    feedback["payload"]["scene"]["obstacles"] = {obstacle};
    store.Consume(ego_topic, feedback.dump());
    assert(!store.GetInput().input_validation().error().empty());
    assert(store.GetInput().scene().version() == 11);
  }
  for (const char* missing : {"target_x", "slot_point4_y", "boundary_line_x1", "need_hybrid_algorithm_fusion"}) {
    auto message = Envelope(request_topic);
    message["payload"].erase(missing);
    InvalidRequest(message);
  }
  for (const json bad : {json("5.5"), json(true), json(nullptr), json(-1.0)}) {
    auto message = Envelope(request_topic);
    message["payload"]["r_min"] = bad;
    InvalidRequest(message);
  }
  for (const json bad : {json(1.5), json(true), json(99), json(-1)}) {
    auto message = Envelope(request_topic);
    message["payload"]["slot_entry"] = bad;
    InvalidRequest(message);
  }
  {
    auto message = Envelope(request_topic);
    message["payload"]["boundary_line_num"] = 1;
    message["payload"]["boundary_line_x1"] = {1.0};
    message["payload"]["boundary_line_y1"] = {2.0};
    message["payload"]["boundary_line_x2"] = {3.0};
    message["payload"]["boundary_line_y2"] = {"bad"};
    InvalidRequest(message);
    message["payload"]["boundary_line_y2"] = json::array();
    InvalidRequest(message);
  }
  {
    auto message = Envelope(request_topic);
    message["payload"]["BoundaryLineX1"] = {1.0};
    InvalidRequest(message);
    message = Envelope(request_topic);
    message["payload"]["boundary_line_num"] = 65;
    InvalidRequest(message);
    message = Envelope(request_topic);
    message["payload"]["goal_tolerance"] = {{"position_m", 0.1}};
    InvalidRequest(message);
  }
  {
    auto store = Ready();
    auto future = Envelope(request_topic, 2, 11, 1.02);
    future["payload"]["target_x"] = 4.0;
    assert(!store.Consume(request_topic, future.dump()));
    assert(store.GetInput().task_id() == 1 && store.GetInput().target_x() == 5.0);
    store.Consume(ego_topic, Envelope(ego_topic, 2, 11, 1.02).dump());
    assert(store.GetInput().task_id() == 2 && store.GetInput().target_x() == 4.0);
    store.Consume(request_topic, Envelope(request_topic).dump());
    assert(store.GetInput().task_id() == 2 && store.GetInput().input_validation().error().empty());
  }
  {
    auto store = Ready();
    auto broken = Envelope(request_topic, 2, 11, 1.02);
    broken["payload"].erase("target_x");
    store.Consume(request_topic, broken.dump());
    store.Consume(request_topic, Envelope(request_topic, 3, 12, 1.04).dump());
    store.Consume(request_topic, Envelope(request_topic, 2, 11, 1.02).dump());
    store.Consume(ego_topic, Envelope(ego_topic, 2, 11, 1.02).dump());
    assert(store.GetInput().input_validation().error() == "missing:target_x");
  }
  {
    auto store = Ready();
    auto broken = Envelope(request_topic, 1, 11, 1.02);
    broken["payload"].erase("target_x");
    store.Consume(request_topic, broken.dump());
    assert(store.GetInput().carla_frame_id() == 10);
    assert(store.GetInput().sim_time_sec() == 1.0);
    assert(store.GetInput().input_validation().error() == "missing:target_x");
  }
  {
    auto store = Ready();
    auto changed = Envelope(ego_topic);
    changed["payload"]["vehicle_x"] = 0.1;
    store.Consume(ego_topic, changed.dump());
    assert(!store.GetInput().input_validation().feedback_valid());
    assert(store.GetInput().vehicle_x() == 0.0);
    assert(store.GetInput().input_validation().error() == "feedback_changed_within_frame");
  }
  {
    auto store = Ready();
    auto safety = Envelope(ego_topic);
    safety["payload"]["safety_state"] = {{"event_id", 1}, {"state", "braking"},
        {"reason", "collision"}, {"standstill_sec", 0.0}};
    store.Consume(ego_topic, safety.dump());
    store.Consume(ego_topic, Envelope(ego_topic).dump());
    assert(store.GetInput().safety_state().reason() == "collision");
    assert(store.GetInput().input_validation().error().empty());
  }
  {
    auto store = Ready();
    auto changed = Envelope(request_topic);
    changed["payload"]["target_x"] = 4.0;
    InvalidRequest(changed);
    changed = Envelope(request_topic, 1, 11, 1.02);
    changed["payload"]["vehicle_x"] = 0.1;
    store.Consume(request_topic, changed.dump());
    assert(store.GetInput().vehicle_x() == 0.0 && store.GetInput().input_validation().error().empty());
  }
  {
    auto store = Ready();
    store.Consume(ego_topic, Envelope(ego_topic, 1, 20, 2.0, "session-b").dump());
    store.Consume(ego_topic, Envelope(ego_topic).dump());
    assert(store.GetInput().session_id() == "session-b");
    assert(store.GetInput().input_validation().error().empty());
  }
  {
    auto store = Ready();
    store.Consume(request_topic, Envelope(request_topic, 1, 11, 1.0).dump());
    assert(!store.GetInput().input_validation().error().empty());
    store = Ready();
    store.Consume(ego_topic, Envelope(ego_topic, 1, 11, 0.98).dump());
    assert(store.GetInput().input_validation().error() == "feedback_clock_discontinuity");
  }
  for (const auto& reason : {"multipart_missing_payload", "multipart_extra_frames"}) {
    auto store = Ready();
    store.RejectMessage(ego_topic, reason);
    assert(store.GetInput().input_validation().error() == reason);
  }
  {
    auto store = Ready();
    store.Consume(ego_topic, "{\"payload\":{},\"payload\":{}}");
    assert(store.GetInput().input_validation().error() == "duplicate_json_key");
    auto direct = Ready().GetInput();
    std::string error;
    direct.set_vehicle_speed(std::numeric_limits<double>::quiet_NaN());
    assert(!pangu::modules::ValidateApaFeedback(direct, &error));
    direct = Ready().GetInput();
    direct.add_boundary_line_x1(1.0);
    assert(!pangu::modules::ValidateApaRequest(direct, &error));
  }
}
