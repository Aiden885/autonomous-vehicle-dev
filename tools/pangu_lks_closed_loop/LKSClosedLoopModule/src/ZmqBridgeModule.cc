#include "ZmqBridgeModule.h"

#include <google/protobuf/text_format.h>

namespace pangu {
namespace modules {

REGISTER_NODE_DEFINE(ZmqBridgeModule);
REGISTER_CALLBACK(ZmqBridgeModule, lks_output, pangu::modules::LksOutput);

void ZmqBridgeModule::SetVersion() {
  SETVERSION(module_name_, "1.0.0");
  SHOWVERSION();
}

bool ZmqBridgeModule::Init(const std::string& ptstr) {
  SetVersion();
  if (!LoadConfig(ptstr) || !InitIO()) {
    return false;
  }
  bridge_ = std::make_unique<LksZmqBridge>();
  if (!bridge_->Init()) {
    return false;
  }
  CREATE_TIMER_TASK(ZmqBridgeModule, params_.timer_duration(), ZmqBridgeModule::Proc);
  return true;
}

bool ZmqBridgeModule::InitIO() {
  CREATE_OUTPUT_RETURN_IF_ERROR(lks_input, pangu::modules::LksInput);
  CREATE_INPUT_RETURN_IF_ERROR(ZmqBridgeModule, lks_output, pangu::modules::LksOutput);
  return true;
}

bool ZmqBridgeModule::LoadConfig(const std::string& ptstr) {
  return google::protobuf::TextFormat::ParseFromString(ptstr, &params_);
}

bool ZmqBridgeModule::Proc() {
  if (!bridge_ || !bridge_->Poll(params_.zmq_poll_timeout_ms())) {
    return false;
  }
  if (!bridge_->HasFreshInput()) {
    return true;
  }
  const auto state = bridge_->GetState();
  if (!state.ego_valid || !state.lane_valid) {
    return true;
  }
  auto input = std::make_shared<pangu::modules::LksInput>();
  input->set_frame_id(frame_id_++);
  input->set_ego_speed_mps(state.ego_speed_mps);
  input->set_c0_m(state.c0_m);
  input->set_c1(state.c1);
  input->set_c2_per_m(state.c2_per_m);
  input->set_c3_per_m2(state.c3_per_m2);
  input->set_curvature_per_m(state.curvature_per_m);
  input->set_brake_pressed(state.brake_pressed);
  input->set_driver_steer_norm(state.driver_steer_norm);
  PUB_MSG(lks_input, input);
  return true;
}

DEFINE_CALLBACK_HEAD(ZmqBridgeModule, lks_output, pangu::modules::LksOutput) {
  std::string inputer;
  auto msg = GET_TRIGGER_MSG_DATA_PROTO(inputer, pangu::modules::LksOutput);
  if (msg == nullptr || !bridge_) {
    return;
  }
  const auto state = bridge_->GetState();
  const bool lateral_enabled = msg->control_enabled() && msg->valid();
  const double manual_steer_rad =
      state.driver_steer_norm * params_.driver_steer_max_rad();
  const double steer_rad = lateral_enabled ? msg->steer_rad() : manual_steer_rad;

  // The LKS algorithm only decides lateral availability. The boundary layer
  // preserves the simulated driver's physical takeover: steering replaces the
  // LKS contribution, while a pressed brake disables the vehicle command and
  // lets CARLA Bridge apply its fail-safe brake.
  const bool vehicle_command_enabled = !state.brake_pressed;
  const double target_speed_mps =
      state.brake_pressed ? 0.0 : params_.target_speed_mps();
  bridge_->PublishControl(target_speed_mps, steer_rad, vehicle_command_enabled);
}

}  // namespace modules
}  // namespace pangu
