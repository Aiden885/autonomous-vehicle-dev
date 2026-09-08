
#include "LksZmqOutputModule_a807407f_21e6_442d_a6e4_13a788ec666d.h"

#include <google/protobuf/text_format.h>


namespace pangu {
namespace modules {REGISTER_NODE_DEFINE(LksZmqOutputModule_a807407f_21e6_442d_a6e4_13a788ec666d);
REGISTER_CALLBACK(LksZmqOutputModule_a807407f_21e6_442d_a6e4_13a788ec666d,
                  lks_output, pangu::modules::LksOutput);

void LksZmqOutputModule_a807407f_21e6_442d_a6e4_13a788ec666d::SetVersion() {
  SETVERSION(module_name_, "1.0.0");
  SHOWVERSION();
}

bool LksZmqOutputModule_a807407f_21e6_442d_a6e4_13a788ec666d::Init(const std::string& ptstr) {
  SetVersion();

  if (!LoadConfig(ptstr)) {
    PLOG_ERROR("Can't load conf file of [{}]", module_name_);
    return false;
  }

  if (!InitIO()) {
    PLOG_ERROR("creat io for {} failed, please check!", module_name_);
    return false;
  }

  zmq_output_bridge_ = std::make_unique<LksZmqBridge>();
  if (!zmq_output_bridge_->Init()) {
    PLOG_ERROR("LksZmqOutputModule_a807407f_21e6_442d_a6e4_13a788ec666d failed to initialize CARLA ZMQ output bridge.");
    return false;
  }
  PLOG_INFO("LksZmqOutputModule_a807407f_21e6_442d_a6e4_13a788ec666d init: target_speed_mps={}, "
            "driver_steer_max_rad={}",
            params_.target_speed_mps(),
            params_.driver_steer_max_rad());
  return true;
}

bool LksZmqOutputModule_a807407f_21e6_442d_a6e4_13a788ec666d::InitIO() {
  CREATE_INPUT_RETURN_IF_ERROR(
      LksZmqOutputModule_a807407f_21e6_442d_a6e4_13a788ec666d, lks_output,
      pangu::modules::LksOutput);

  PLOG_INFO("LksZmqOutputModule_a807407f_21e6_442d_a6e4_13a788ec666d InitIO end");
  return true;
}

bool LksZmqOutputModule_a807407f_21e6_442d_a6e4_13a788ec666d::LoadConfig(const std::string& ptstr) {
  if (!google::protobuf::TextFormat::ParseFromString(ptstr,
                                                     &params_)) {
    PLOG_ERROR("Can't get conf file of [{}]", module_name_);
    return false;
  }

  return true;
}

bool LksZmqOutputModule_a807407f_21e6_442d_a6e4_13a788ec666d::Proc() {
  return true;
}

DEFINE_CALLBACK_HEAD(
    LksZmqOutputModule_a807407f_21e6_442d_a6e4_13a788ec666d, lks_output,
    pangu::modules::LksOutput) {
  std::string inputer;
  auto msg = GET_TRIGGER_MSG_DATA_PROTO(inputer, pangu::modules::LksOutput);
  if (msg == nullptr || !zmq_output_bridge_) {
    return;
  }

  if (!zmq_output_bridge_->Poll(0)) {
    PLOG_WARN("Failed to poll LKS driver state before control output.");
  }
  const auto state = zmq_output_bridge_->GetState();
  const bool lateral_enabled = msg->control_enabled();
  const double manual_steer_rad =
      state.driver_steer_norm * params_.driver_steer_max_rad();
  const double steer_rad = lateral_enabled ? msg->steer_rad() : manual_steer_rad;
  const bool vehicle_command_enabled = !state.brake_pressed;
  const double target_speed_mps =
      state.brake_pressed ? 0.0 : params_.target_speed_mps();

  if (!zmq_output_bridge_->PublishControl(target_speed_mps, steer_rad,
                                          vehicle_command_enabled)) {
    PLOG_WARN("Failed to publish LKS control command to CARLA Bridge.");
  }
}


} // namespace pangu
} // namespace modules
