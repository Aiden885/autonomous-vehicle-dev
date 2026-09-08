
#include "AccZmqInputModule.h"

#include <google/protobuf/text_format.h>


namespace pangu {
namespace modules {REGISTER_NODE_DEFINE(AccZmqInputModule);

void AccZmqInputModule::SetVersion() {
  SETVERSION(module_name_, "1.0.0");
  SHOWVERSION();
}

bool AccZmqInputModule::Init(const std::string& ptstr) {
  SetVersion();

  if (!LoadConfig(ptstr)) {
    PLOG_ERROR("Can't load conf file of [{}]", module_name_);
    return false;
  }

  if (!InitIO()) {
    PLOG_ERROR("creat io for {} failed, please check!", module_name_);
    return false;
  }

  zmq_input_bridge_ = std::make_unique<AccZmqInputBridge>();
  if (!zmq_input_bridge_->Init()) {
    PLOG_ERROR("AccZmqInputModule failed to initialize CARLA ZMQ input bridge.");
    return false;
  }
  frame_id_counter_ = 0;
  CREATE_TIMER_TASK(AccZmqInputModule, acc_zmq_input_params_.timer_duration(),
                    AccZmqInputModule::Proc);
  PLOG_INFO("AccZmqInputModule init: timer_duration={}, "
            "zmq_poll_timeout_ms={}",
            acc_zmq_input_params_.timer_duration(),
            acc_zmq_input_params_.zmq_poll_timeout_ms());
  return true;
}

bool AccZmqInputModule::InitIO() {
    CREATE_OUTPUT_RETURN_IF_ERROR(acc_input, pangu::modules::AccInput);

  PLOG_INFO("AccZmqInputModule InitIO end");
  return true;
}

bool AccZmqInputModule::LoadConfig(const std::string& ptstr) {
  if (!google::protobuf::TextFormat::ParseFromString(ptstr,
                                                     &acc_zmq_input_params_)) {
    PLOG_ERROR("Can't get conf file of [{}]", module_name_);
    return false;
  }

  return true;
}

bool AccZmqInputModule::Proc() {
  if (!zmq_input_bridge_) {
    PLOG_ERROR("AccZmqInputModule Proc called before initialization.");
    return false;
  }

  if (!zmq_input_bridge_->Poll(acc_zmq_input_params_.zmq_poll_timeout_ms())) {
    PLOG_WARN("AccZmqInputModule failed to poll CARLA ZMQ bridge.");
    return false;
  }

  if (!zmq_input_bridge_->HasFreshInput()) {
    return true;
  }

  const auto zmq_state = zmq_input_bridge_->GetState();
  auto input = std::make_shared<pangu::modules::AccInput>();
  input->set_ego_speed_mps(zmq_state.ego_speed_mps);
  input->set_lead_speed_mps(zmq_state.lead_speed_mps);
  input->set_command_type(zmq_state.command_type);
  input->set_frame_id(frame_id_counter_++);
  input->set_lead_distance_m(zmq_state.lead_distance_m);

  PUB_MSG(acc_input, input);
  return true;
}


} // namespace pangu
} // namespace modules