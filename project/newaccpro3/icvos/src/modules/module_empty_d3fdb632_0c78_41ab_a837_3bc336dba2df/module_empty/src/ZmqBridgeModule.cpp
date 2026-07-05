
#include "ZmqBridgeModule.h"

#include <algorithm>
#include <cstdlib>
#include <fstream>
#include <google/protobuf/text_format.h>


namespace pangu {
namespace modules {REGISTER_NODE_DEFINE(ZmqBridgeModule);

namespace {

double ReadEnvDouble(const char* name, double fallback) {
  const char* value = std::getenv(name);
  if (value == nullptr || value[0] == '\0') {
    return fallback;
  }
  char* end = nullptr;
  const double parsed = std::strtod(value, &end);
  if (end == value) {
    return fallback;
  }
  return parsed;
}

void AppendDebugLine(const std::string& text) {
  const char* path = std::getenv("ACC_ZMQ_DEBUG_LOG");
  if (path == nullptr || path[0] == '\0') {
    return;
  }
  std::ofstream out(path, std::ios::app);
  if (out.is_open()) {
    out << text << '\n';
  }
}

}  // namespace

void ZmqBridgeModule::SetVersion() {
  SETVERSION(module_name_, "1.0.0");
  SHOWVERSION();
}

bool ZmqBridgeModule::Init(const std::string& ptstr) {
  SetVersion();

  if (!LoadConfig(ptstr)) {
    PLOG_ERROR("Can't load conf file of [{}]", module_name_);
    return false;
  }

  if (!InitIO()) {
    PLOG_ERROR("creat io for {} failed, please check!", module_name_);
    return false;
  }

  control::composite_block_empty_ea2170e7_297c_4433_b70a_9c66e48988adTraits::Param
      decision_param{};
  decision_param.vMin = ReadEnvDouble("ACC_DECISION_VMIN", 1.0);
  decision_param.GapStep = ReadEnvDouble("ACC_GAP_STEP", 0.2);
  decision_param.MinGap = ReadEnvDouble("ACC_MIN_TIME_GAP", 1.0);
  decision_param.MaxGap = ReadEnvDouble("ACC_MAX_TIME_GAP", 5.0);
  decision_param.SpdStep = ReadEnvDouble("ACC_SPEED_STEP", 4.0);
  decision_param.MinSpd = ReadEnvDouble("ACC_MIN_SPEED", 0.0);
  acc_decision_.setParam(decision_param);

  control::composite_block_empty_ea2170e7_297c_4433_b70a_9c66e48988adTraits::State
      decision_state{};
  decision_state.controlEnabled = 0;
  decision_state.hasHistory = 0;
  decision_state.timeGap = ReadEnvDouble("ACC_INITIAL_TIME_GAP", 1.8);
  decision_state.maxSpeed = ReadEnvDouble("ACC_INITIAL_MAX_SPEED", 20.0);
  acc_decision_.setState(decision_state);

  control::global::params.MinDistance =
      ReadEnvDouble("ACC_MIN_DISTANCE", control::global::params.MinDistance);
  control::global::params.Kdist =
      ReadEnvDouble("ACC_DISTANCE_GAIN", control::global::params.Kdist);
  control::global::params.Kspeed =
      ReadEnvDouble("ACC_SPEED_GAIN", control::global::params.Kspeed);

  zmq_bridge_ = std::make_unique<AccZmqBridge>();
  if (!zmq_bridge_->Init()) {
    PLOG_ERROR("ZmqBridgeModule failed to initialize CARLA ZMQ bridge.");
    return false;
  }

  frame_id_counter_ = 0;
  CREATE_TIMER_TASK(ZmqBridgeModule, zmq_bridge_params_.timer_duration(),
                    ZmqBridgeModule::Proc);

  PLOG_INFO("ZmqBridgeModule init: timer_duration={}, zmq_poll_timeout_ms={}, "
            "zmq_control_enable={}",
            zmq_bridge_params_.timer_duration(),
            zmq_bridge_params_.zmq_poll_timeout_ms(),
            zmq_bridge_params_.zmq_control_enable());
  return true;
}

bool ZmqBridgeModule::InitIO() {

  PLOG_INFO("in InitIO end");
  return true;
}

bool ZmqBridgeModule::LoadConfig(const std::string& ptstr) {
  if (!google::protobuf::TextFormat::ParseFromString(ptstr,
                                                     &zmq_bridge_params_)) {
    PLOG_ERROR("Can't get conf file of [{}]", module_name_);
    return false;
  }

  return true;
}

bool ZmqBridgeModule::Proc() {
  if (!zmq_bridge_) {
    PLOG_ERROR("ZmqBridgeModule Proc called before initialization.");
    return false;
  }

  if (!zmq_bridge_->Poll(zmq_bridge_params_.zmq_poll_timeout_ms())) {
    PLOG_WARN("ZmqBridgeModule failed to poll CARLA ZMQ bridge.");
    return false;
  }

  if (!zmq_bridge_->HasFreshInput()) {
    return true;
  }

  const auto zmq_state = zmq_bridge_->GetState();
  control::composite_block_empty_ea2170e7_297c_4433_b70a_9c66e48988adTraits::Input
      decision_input{};
  decision_input.commandType = zmq_state.command_type;
  decision_input.egoV = zmq_state.ego_speed_mps;

  control::composite_block_empty_ea2170e7_297c_4433_b70a_9c66e48988adTraits::Output
      decision_output{};
  acc_decision_.run(decision_input, decision_output);

  double target_speed = 0.0;
  if (zmq_state.ego_valid && zmq_state.lead_valid) {
    const double relative_speed =
        zmq_state.lead_speed_mps - zmq_state.ego_speed_mps;
    const double desired_distance =
        std::max(zmq_state.ego_speed_mps * decision_output.timeGap,
                 control::global::params.MinDistance);
    const double distance_error = zmq_state.lead_distance_m - desired_distance;
    const double raw_target =
        zmq_state.lead_speed_mps +
        control::global::params.Kdist * distance_error +
        control::global::params.Kspeed * relative_speed;
    target_speed =
        std::min(std::max(raw_target, 0.0), decision_output.maxSpeed);
  }

  if (!zmq_bridge_->PublishControl(target_speed, decision_output.enable)) {
    PLOG_WARN("ZmqBridgeModule failed to publish ACC control command.");
    return false;
  }

  AppendDebugLine(
      "egoV=" + std::to_string(zmq_state.ego_speed_mps) +
      " leadV=" + std::to_string(zmq_state.lead_speed_mps) +
      " distance=" + std::to_string(zmq_state.lead_distance_m) +
      " commandType=" + std::to_string(zmq_state.command_type) +
      " commandValid=" + std::to_string(zmq_state.command_valid ? 1 : 0) +
      " enable=" + std::to_string(decision_output.enable) +
      " timeGap=" + std::to_string(decision_output.timeGap) +
      " maxSpeed=" + std::to_string(decision_output.maxSpeed) +
      " target=" + std::to_string(target_speed));

  return true;
}


} // namespace pangu
} // namespace modules
