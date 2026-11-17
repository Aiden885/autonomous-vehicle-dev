#include "apa_zmq_bridge.h"

#include <zmq.h>
#include <nlohmann/json.hpp>

#include <chrono>
#include <cmath>
#include <cstdlib>
#include <cstring>

namespace pangu::modules {
namespace {

using Json = nlohmann::json;

constexpr const char* kProtocol = "gaasd_carla_bridge";
constexpr const char* kProtocolVersion = kApaInputProtocolVersion;
constexpr const char* kPlanRequestTopic = "gaasd.carla.apa_plan_request.v1";
constexpr const char* kEgoStateTopic = "gaasd.carla.ego_state.v1";
constexpr const char* kPlanResultTopic = "gaasd.carla.apa_plan_result.v1";
constexpr const char* kControlTopic = "gaasd.carla.control_cmd.v1";

std::string EnvString(const char* name, const char* fallback) {
  const char* value = std::getenv(name);
  if (value == nullptr || value[0] == '\0') {
    return std::string(fallback);
  }
  return std::string(value);
}

long long UnixMs() {
  using Clock = std::chrono::system_clock;
  return std::chrono::duration_cast<std::chrono::milliseconds>(
             Clock::now().time_since_epoch())
      .count();
}

struct ZmqMessage {
  zmq_msg_t value;
  ZmqMessage() { zmq_msg_init(&value); }
  ~ZmqMessage() { zmq_msg_close(&value); }
};

void SetLingerZero(void* socket) {
  const int linger = 0;
  (void)zmq_setsockopt(socket, ZMQ_LINGER, &linger, sizeof(linger));
}

bool Subscribe(void* socket, const char* topic) {
  return zmq_setsockopt(socket, ZMQ_SUBSCRIBE, topic, std::strlen(topic)) == 0;
}

Json Header(const char* topic, unsigned long long sequence,
            const std::string& source) {
  return Json{{"protocol", kProtocol},
              {"protocol_version", kProtocolVersion},
              {"message_type", topic},
              {"sequence", sequence},
              {"timestamp_unix_ms", UnixMs()},
              {"source", source}};
}

}

ApaZmqInputBridge::ApaZmqInputBridge()
    : input_endpoint_(EnvString("GAASD_CARLA_INPUT_ENDPOINT",
                                "tcp://127.0.0.1:5701")) {}

ApaZmqInputBridge::~ApaZmqInputBridge() { Shutdown(); }

bool ApaZmqInputBridge::Init() {
  if (initialized_) {
    return true;
  }
  context_ = zmq_ctx_new();
  if (context_ == nullptr) {
    return false;
  }
  sub_ = zmq_socket(context_, ZMQ_SUB);
  if (sub_ == nullptr) {
    Shutdown();
    return false;
  }
  SetLingerZero(sub_);
  if (!Subscribe(sub_, kPlanRequestTopic) || !Subscribe(sub_, kEgoStateTopic)) {
    Shutdown();
    return false;
  }
  if (zmq_connect(sub_, input_endpoint_.c_str()) != 0) {
    Shutdown();
    return false;
  }
  initialized_ = true;
  return true;
}

void ApaZmqInputBridge::Shutdown() {
  if (sub_ != nullptr) {
    zmq_close(sub_);
    sub_ = nullptr;
  }
  if (context_ != nullptr) {
    zmq_ctx_term(context_);
    context_ = nullptr;
  }
  initialized_ = false;
}

bool ApaZmqInputBridge::Poll(int timeout_ms) {
  if (!Init()) {
    return false;
  }
  fresh_input_ = false;
  zmq_pollitem_t items[] = {{sub_, 0, ZMQ_POLLIN, 0}};
  const int poll_rc = zmq_poll(items, 1, timeout_ms < 0 ? 0 : timeout_ms);
  if (poll_rc < 0) {
    return false;
  }
  if ((items[0].revents & ZMQ_POLLIN) == 0) {
    return true;
  }

  for (int i = 0; i < 100; ++i) {
    ZmqMessage topic_msg;
    ZmqMessage payload_msg;
    if (zmq_msg_recv(&topic_msg.value, sub_, ZMQ_DONTWAIT) < 0) break;
    const std::string topic(static_cast<const char*>(zmq_msg_data(&topic_msg.value)),
                            zmq_msg_size(&topic_msg.value));
    if (!zmq_msg_more(&topic_msg.value)) {
      fresh_input_ |= input_contract_.RejectMessage(topic, "multipart_missing_payload");
      continue;
    }
    if (zmq_msg_recv(&payload_msg.value, sub_, ZMQ_DONTWAIT) < 0) {
      fresh_input_ |= input_contract_.RejectMessage(topic, "multipart_incomplete");
      break;
    }
    if (zmq_msg_more(&payload_msg.value)) {
      fresh_input_ |= input_contract_.RejectMessage(topic, "multipart_extra_frames");
      int extra = 0;
      while (zmq_msg_more(&payload_msg.value)) {
        if (++extra > 100 || zmq_msg_recv(&payload_msg.value, sub_, ZMQ_DONTWAIT) < 0) {

          Shutdown();
          return true;
        }
      }
      continue;
    }
    const std::string payload(static_cast<const char*>(zmq_msg_data(&payload_msg.value)),
                              zmq_msg_size(&payload_msg.value));
    fresh_input_ |= input_contract_.Consume(topic, payload);
  }
  return true;
}

bool ApaZmqInputBridge::HasFreshInput() const { return fresh_input_; }

ApaInput ApaZmqInputBridge::GetInput() const { return input_contract_.GetInput(); }

ApaZmqOutputBridge::ApaZmqOutputBridge()
    : output_endpoint_(EnvString("GAASD_CARLA_OUTPUT_ENDPOINT",
                                 "tcp://127.0.0.1:5702")),
      source_(EnvString("GAASD_CARLA_SOURCE", "pangu_apa_module")) {}

ApaZmqOutputBridge::~ApaZmqOutputBridge() { Shutdown(); }

bool ApaZmqOutputBridge::Init() {
  if (initialized_) {
    return true;
  }
  context_ = zmq_ctx_new();
  if (context_ == nullptr) {
    return false;
  }
  pub_ = zmq_socket(context_, ZMQ_PUB);
  if (pub_ == nullptr) {
    Shutdown();
    return false;
  }
  SetLingerZero(pub_);
  if (zmq_bind(pub_, output_endpoint_.c_str()) != 0) {
    Shutdown();
    return false;
  }
  initialized_ = true;
  return true;
}

void ApaZmqOutputBridge::Shutdown() {
  if (pub_ != nullptr) {
    zmq_close(pub_);
    pub_ = nullptr;
  }
  if (context_ != nullptr) {
    zmq_ctx_term(context_);
    context_ = nullptr;
  }
  initialized_ = false;
}

bool ApaZmqOutputBridge::PublishJson(const char* topic, const std::string& text) {
  if (zmq_send(pub_, topic, std::strlen(topic), ZMQ_SNDMORE) < 0) {
    return false;
  }
  return zmq_send(pub_, text.data(), text.size(), 0) >= 0;
}

bool ApaZmqOutputBridge::Publish(const ApaOutput& output) {
  std::lock_guard<std::mutex> lock(publish_mutex_);
  if (!Init()) {
    return false;
  }

  Json path_x = Json::array();
  Json path_y = Json::array();
  Json path_yaw = Json::array();
  Json path_odo = Json::array();
  for (int i = 0; i < output.path_point_x_size(); ++i) path_x.push_back(output.path_point_x(i));
  for (int i = 0; i < output.path_point_y_size(); ++i) path_y.push_back(output.path_point_y(i));
  for (int i = 0; i < output.path_point_yaw_size(); ++i) path_yaw.push_back(output.path_point_yaw(i));
  for (int i = 0; i < output.path_point_odo_size(); ++i) path_odo.push_back(output.path_point_odo(i));
  const auto& d = output.control_diagnostic();
  const auto& safety = output.safety_state();
  const auto& validation = output.input_validation();
  const auto& online = output.online_safety();
  const auto& replan = output.replan();
  const Json replan_json{{"recovery_id", replan.recovery_id()}, {"path_version", replan.path_version()},
      {"state", replan.state()}, {"attempt", replan.attempt()}, {"reason", replan.reason()},
      {"elapsed_sec", replan.elapsed_sec()}, {"planner_error_type", replan.planner_error_type()}};
  const Json online_json{{"blocked", online.blocked()}, {"scene_version", online.scene_version()},
      {"plan_scene_version", online.plan_scene_version()}, {"reason", online.reason()},
      {"obstacle_id", online.obstacle_id()}, {"checked_distance_m", online.checked_distance_m()}};
  const Json validation_json{{"feedback_valid", validation.feedback_valid()},
      {"request_valid", validation.request_valid()}, {"error", validation.error()}};
  const Json safety_json{{"event_id", safety.event_id()}, {"reason", safety.reason()},
      {"state", safety.state()}, {"standstill_sec", safety.standstill_sec()}};
  const Json diagnostic{{"segment", d.segment()}, {"segment_count", d.segment_count()},
      {"reference_index", d.reference_index()}, {"remaining_distance_m", d.remaining_distance_m()},
      {"stop_distance_m", d.stop_distance_m()}, {"cross_track_error_m", d.cross_track_error_m()},
      {"heading_error_rad", d.heading_error_rad()}, {"endpoint_distance_m", d.endpoint_distance_m()},
      {"endpoint_yaw_error_rad", d.endpoint_yaw_error_rad()}, {"vehicle_speed_mps", d.vehicle_speed_mps()},
      {"vehicle_acceleration_mps2", d.vehicle_acceleration_mps2()}, {"feedback_gear", d.feedback_gear()},
      {"stop_reason", d.stop_reason()}, {"reference_valid", d.reference_valid()},
      {"steer_pursuit_rad", d.steer_pursuit_rad()}, {"steer_heading_rad", d.steer_heading_rad()},
      {"steer_saturated", d.steer_saturated()}, {"progress_m", d.progress_m()},
      {"reference_x", d.reference_x()}, {"reference_y", d.reference_y()},
      {"goal_x", d.goal_x()}, {"goal_y", d.goal_y()}, {"goal_yaw_rad", d.goal_yaw_rad()}};

  const auto plan_seq = sequence_++;
  const auto& limits = output.actuation_limits();
  const Json limits_json{{"max_abs_target_speed_mps", limits.max_abs_target_speed_mps()},
      {"max_abs_accel_mps2", limits.max_abs_accel_mps2()}, {"max_abs_steer_rad", limits.max_abs_steer_rad()}};
  const Json plan_msg{{"header", Header(kPlanResultTopic, plan_seq, source_)},
                      {"payload",
                       Json{{"frame_id", output.frame_id()},
                            {"vehicle_profile_id", output.vehicle_profile_id()},
                            {"session_id", output.session_id()},
                            {"task_id", output.task_id()},
                            {"carla_frame_id", output.carla_frame_id()},
                            {"sim_time_sec", output.sim_time_sec()},
                            {"control_status", output.control_status()},
                            {"planning_only", output.planning_only()},
                            {"safety_state", safety_json},
                            {"input_validation", validation_json},
                            {"online_safety", online_json},
                            {"replan", replan_json},
                            {"actuation_limits", limits_json},
                            {"success", output.path_point_x_size() > 1},
                            {"path_point_x", path_x},
                            {"path_point_y", path_y},
                            {"path_point_yaw", path_yaw},
                            {"path_point_odo", path_odo},
                            {"PthPtX", path_x},
                            {"PthPtY", path_y},
                            {"PthPtYaw", path_yaw}}}};

  const auto control_seq = sequence_++;
  const Json control_msg{
      {"header", Header(kControlTopic, control_seq, source_)},
      {"payload",
       Json{{"command_id", control_seq},
            {"vehicle_profile_id", output.vehicle_profile_id()},
            {"frame_id", output.frame_id()},
            {"session_id", output.session_id()},
            {"task_id", output.task_id()},
            {"carla_frame_id", output.carla_frame_id()},
            {"sim_time_sec", output.sim_time_sec()},
            {"control_dt_sec", output.control_dt_sec()},
            {"control_status", output.control_status()},
            {"planning_only", output.planning_only()},
            {"safety_state", safety_json},
            {"input_validation", validation_json},
            {"online_safety", online_json},
            {"replan", replan_json},
            {"actuation_limits", limits_json},
            {"actuator", Json{{"owner", output.actuator().owner()},
                {"throttle", output.actuator().throttle()}, {"brake", output.actuator().brake()},
                {"hand_brake", output.actuator().hand_brake()}, {"gear_pos", output.actuator().gear_pos()},
                {"launch_hold", output.actuator().launch_hold()}}},
            {"control_step_count", output.control_step_count()},
            {"diagnostic", diagnostic},
            {"enable", output.enable()},
            {"StopReq", output.stop_req()},
            {"TargetSpeed", output.target_speed_mps()},
            {"TargetAcceleration", output.target_accel_mps2()},
            {"FrontWheelAngle", output.steer_rad()},
            {"TargetGearReq", output.target_gear_req()},
            {"DirectionSign", output.direction_sign()},
            {"GearChangeAllowed", output.gear_change_allowed()},
            {"CtrlPhase", output.ctrl_phase()},
            {"target",
             Json{{"target_speed_mps", output.target_speed_mps()},
                  {"target_accel_mps2", output.target_accel_mps2()},
                  {"steer_rad", output.steer_rad()}}}}}};

  return PublishJson(kPlanResultTopic, plan_msg.dump(-1, ' ', false, Json::error_handler_t::replace)) &&
         PublishJson(kControlTopic, control_msg.dump(-1, ' ', false, Json::error_handler_t::replace));
}

}
