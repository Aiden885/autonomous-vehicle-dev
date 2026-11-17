
#include "AccZmqInputModule_b6d54e1b_2577_4492_bcea_dfe707509e43.h"

#include <google/protobuf/text_format.h>


namespace pangu {
namespace modules {REGISTER_NODE_DEFINE(AccZmqInputModule_b6d54e1b_2577_4492_bcea_dfe707509e43);

void AccZmqInputModule_b6d54e1b_2577_4492_bcea_dfe707509e43::SetVersion() {
  SETVERSION(module_name_, "1.0.0");
  SHOWVERSION();
}

bool AccZmqInputModule_b6d54e1b_2577_4492_bcea_dfe707509e43::Init(const std::string& ptstr) {
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
    PLOG_ERROR("AccZmqInputModule_b6d54e1b_2577_4492_bcea_dfe707509e43 failed to initialize CARLA ZMQ input bridge.");
    return false;
  }
  frame_id_counter_ = 0;
  CREATE_TIMER_TASK(AccZmqInputModule_b6d54e1b_2577_4492_bcea_dfe707509e43, acc_zmq_input_params_.timer_duration(),
                    AccZmqInputModule_b6d54e1b_2577_4492_bcea_dfe707509e43::Proc);
  PLOG_INFO("AccZmqInputModule_b6d54e1b_2577_4492_bcea_dfe707509e43 init: timer_duration={}, "
            "zmq_poll_timeout_ms={}",
            acc_zmq_input_params_.timer_duration(),
            acc_zmq_input_params_.zmq_poll_timeout_ms());
  return true;
}

bool AccZmqInputModule_b6d54e1b_2577_4492_bcea_dfe707509e43::InitIO() {
  CREATE_OUTPUT_RETURN_IF_ERROR(acc_input, pangu::modules::AccInput);

  PLOG_INFO("AccZmqInputModule_b6d54e1b_2577_4492_bcea_dfe707509e43 InitIO end");
  return true;
}

bool AccZmqInputModule_b6d54e1b_2577_4492_bcea_dfe707509e43::LoadConfig(const std::string& ptstr) {
  if (!google::protobuf::TextFormat::ParseFromString(ptstr,
                                                     &acc_zmq_input_params_)) {
    PLOG_ERROR("Can't get conf file of [{}]", module_name_);
    return false;
  }

  return true;
}

bool AccZmqInputModule_b6d54e1b_2577_4492_bcea_dfe707509e43::Proc() {
  if (!zmq_input_bridge_) {
    PLOG_ERROR("AccZmqInputModule_b6d54e1b_2577_4492_bcea_dfe707509e43 Proc called before initialization.");
    return false;
  }

  if (!zmq_input_bridge_->Poll(acc_zmq_input_params_.zmq_poll_timeout_ms())) {
    PLOG_WARN("AccZmqInputModule_b6d54e1b_2577_4492_bcea_dfe707509e43 failed to poll CARLA ZMQ bridge.");
    return false;
  }

  if (!zmq_input_bridge_->HasFreshInput()) {
    return true;
  }

  const auto zmq_state = zmq_input_bridge_->GetState();
  if (!zmq_state.ego_valid || !zmq_state.lead_valid) {
    return true;
  }

  auto input = std::make_shared<pangu::modules::AccInput>();
  input->set_frame_id(frame_id_counter_++);
  input->set_ego_speed_mps(zmq_state.ego_speed_mps);
  input->set_lead_speed_mps(zmq_state.lead_speed_mps);
  input->set_lead_distance_m(zmq_state.lead_distance_m);
  input->set_command_type(zmq_state.command_type);
  PUB_MSG(acc_input, input);
  return true;
}


} // namespace pangu
} // namespace modules
