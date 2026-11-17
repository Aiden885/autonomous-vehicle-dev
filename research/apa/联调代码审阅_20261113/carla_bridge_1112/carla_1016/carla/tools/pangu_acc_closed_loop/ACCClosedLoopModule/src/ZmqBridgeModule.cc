#include "ZmqBridgeModule.h"

#include <google/protobuf/text_format.h>

namespace pangu {
namespace modules {

REGISTER_NODE_DEFINE(ZmqBridgeModule);
REGISTER_CALLBACK(ZmqBridgeModule, acc_output, pangu::modules::AccOutput);

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

  zmq_bridge_ = std::make_unique<AccZmqBridge>();
  if (!zmq_bridge_->Init()) {
    PLOG_ERROR("ZmqBridgeModule failed to initialize CARLA ZMQ bridge.");
    return false;
  }

  frame_id_counter_ = 0;
  CREATE_TIMER_TASK(ZmqBridgeModule, ZmqBridgeModule_params_.timer_duration(),
                    ZmqBridgeModule::Proc);

  PLOG_INFO("ZmqBridgeModule init: timer_duration={}, zmq_poll_timeout_ms={}, "
            "zmq_control_enable={}",
            ZmqBridgeModule_params_.timer_duration(),
            ZmqBridgeModule_params_.zmq_poll_timeout_ms(),
            ZmqBridgeModule_params_.zmq_control_enable());
  return true;
}

bool ZmqBridgeModule::InitIO() {
  CREATE_OUTPUT_RETURN_IF_ERROR(acc_input, pangu::modules::AccInput);
  CREATE_INPUT_RETURN_IF_ERROR(ZmqBridgeModule, acc_output,
                               pangu::modules::AccOutput);

  PLOG_INFO("in InitIO end");
  return true;
}

bool ZmqBridgeModule::LoadConfig(const std::string& ptstr) {
  if (!google::protobuf::TextFormat::ParseFromString(ptstr,
                                                     &ZmqBridgeModule_params_)) {
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

  if (!zmq_bridge_->Poll(ZmqBridgeModule_params_.zmq_poll_timeout_ms())) {
    PLOG_WARN("ZmqBridgeModule failed to poll CARLA ZMQ bridge.");
    return false;
  }

  if (!zmq_bridge_->HasFreshInput()) {
    return true;
  }

  const auto zmq_state = zmq_bridge_->GetState();
  if (!zmq_state.ego_valid || !zmq_state.lead_valid) {
    PLOG_DEBUG("ZmqBridgeModule waiting for fresh ZMQ data, ego_valid:{}, "
               "lead_valid:{}",
               zmq_state.ego_valid, zmq_state.lead_valid);
    return true;
  }

  auto input = std::make_shared<pangu::modules::AccInput>();
  input->set_frame_id(frame_id_counter_++);
  input->set_ego_speed_mps(zmq_state.ego_speed_mps);
  input->set_lead_speed_mps(zmq_state.lead_speed_mps);
  input->set_lead_distance_m(zmq_state.lead_distance_m);
  input->set_command_type(zmq_state.command_type);

  PUB_MSG(acc_input, input);
  PLOG_INFO("ZmqBridgeModule pub acc_input frame_id:{}, ego_speed_mps:{}, "
            "lead_speed_mps:{}, lead_distance_m:{}, command_type:{}",
            input->frame_id(), input->ego_speed_mps(), input->lead_speed_mps(),
            input->lead_distance_m(), input->command_type());
  return true;
}

DEFINE_CALLBACK_HEAD(ZmqBridgeModule, acc_output, pangu::modules::AccOutput) {
  PLOG_INFO("get acc_output");
  std::string inputer;
  auto msg = GET_TRIGGER_MSG_DATA_PROTO(inputer, pangu::modules::AccOutput);
  if (msg == nullptr) {
    PLOG_WARN("acc_output callback got null message from {}", inputer);
    return;
  }

  PLOG_DEBUG("input:{} acc_output: {}", inputer, msg->ShortDebugString());
  if (!zmq_bridge_) {
    PLOG_ERROR("ZmqBridgeModule acc_output callback before ZMQ init.");
    return;
  }

  if (!zmq_bridge_->PublishControl(msg->target_speed_mps(),
                                   msg->enable() ? 1 : 0)) {
    PLOG_WARN("ZmqBridgeModule failed to publish CARLA ZMQ control command.");
  }
}

}  // namespace modules
}  // namespace pangu
