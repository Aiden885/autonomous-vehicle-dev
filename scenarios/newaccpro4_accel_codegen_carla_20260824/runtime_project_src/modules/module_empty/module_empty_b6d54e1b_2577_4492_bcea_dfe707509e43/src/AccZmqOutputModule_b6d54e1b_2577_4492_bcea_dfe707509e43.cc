
#include "AccZmqOutputModule_b6d54e1b_2577_4492_bcea_dfe707509e43.h"

#include <google/protobuf/text_format.h>


namespace pangu {
namespace modules {REGISTER_NODE_DEFINE(AccZmqOutputModule_b6d54e1b_2577_4492_bcea_dfe707509e43);
REGISTER_CALLBACK(AccZmqOutputModule_b6d54e1b_2577_4492_bcea_dfe707509e43,
                  acc_output, pangu::modules::AccOutput);

void AccZmqOutputModule_b6d54e1b_2577_4492_bcea_dfe707509e43::SetVersion() {
  SETVERSION(module_name_, "1.0.0");
  SHOWVERSION();
}

bool AccZmqOutputModule_b6d54e1b_2577_4492_bcea_dfe707509e43::Init(const std::string& ptstr) {
  SetVersion();

  if (!LoadConfig(ptstr)) {
    PLOG_ERROR("Can't load conf file of [{}]", module_name_);
    return false;
  }

  if (!InitIO()) {
    PLOG_ERROR("creat io for {} failed, please check!", module_name_);
    return false;
  }

  zmq_output_bridge_ = std::make_unique<AccZmqOutputBridge>();
  if (!zmq_output_bridge_->Init()) {
    PLOG_ERROR("AccZmqOutputModule_b6d54e1b_2577_4492_bcea_dfe707509e43 failed to initialize CARLA ZMQ output bridge.");
    return false;
  }
  PLOG_INFO("AccZmqOutputModule_b6d54e1b_2577_4492_bcea_dfe707509e43 init: zmq_control_enable={}",
            acc_zmq_output_params_.zmq_control_enable());
  return true;
}

bool AccZmqOutputModule_b6d54e1b_2577_4492_bcea_dfe707509e43::InitIO() {
  CREATE_INPUT_RETURN_IF_ERROR(
      AccZmqOutputModule_b6d54e1b_2577_4492_bcea_dfe707509e43, acc_output,
      pangu::modules::AccOutput);

  PLOG_INFO("AccZmqOutputModule_b6d54e1b_2577_4492_bcea_dfe707509e43 InitIO end");
  return true;
}

bool AccZmqOutputModule_b6d54e1b_2577_4492_bcea_dfe707509e43::LoadConfig(const std::string& ptstr) {
  if (!google::protobuf::TextFormat::ParseFromString(ptstr,
                                                     &acc_zmq_output_params_)) {
    PLOG_ERROR("Can't get conf file of [{}]", module_name_);
    return false;
  }

  return true;
}

bool AccZmqOutputModule_b6d54e1b_2577_4492_bcea_dfe707509e43::Proc() {
  return true;
}

DEFINE_CALLBACK_HEAD(
    AccZmqOutputModule_b6d54e1b_2577_4492_bcea_dfe707509e43, acc_output,
    pangu::modules::AccOutput) {
  std::string inputer;
  auto msg = GET_TRIGGER_MSG_DATA_PROTO(inputer, pangu::modules::AccOutput);
  if (msg == nullptr || !zmq_output_bridge_) {
    return;
  }

  const int enable =
      acc_zmq_output_params_.zmq_control_enable() && msg->enable() ? 1 : 0;
  const double acceleration_command_mps2 = msg->target_speed_mps();
  if (!zmq_output_bridge_->PublishAccelerationControl(
          acceleration_command_mps2, enable)) {
    PLOG_WARN("Failed to publish ACC acceleration command to CARLA Bridge.");
  }
}


} // namespace pangu
} // namespace modules
