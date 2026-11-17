#include "ApaZmqOutputModule.h"

#include <google/protobuf/text_format.h>

namespace pangu {
namespace modules {

REGISTER_NODE_DEFINE(ApaZmqOutputModule);
REGISTER_CALLBACK(ApaZmqOutputModule, apa_output, pangu::modules::ApaOutput);

void ApaZmqOutputModule::SetVersion() {
  SETVERSION(module_name_, "1.0.0");
  SHOWVERSION();
}

bool ApaZmqOutputModule::Init(const std::string& ptstr) {
  SetVersion();
  if (!LoadConfig(ptstr)) {
    PLOG_ERROR("Can't load conf file of [{}]", module_name_);
    return false;
  }
  if (!InitIO()) {
    PLOG_ERROR("creat io for {} failed, please check!", module_name_);
    return false;
  }

  zmq_output_bridge_ = std::make_unique<ApaZmqOutputBridge>();
  if (!zmq_output_bridge_->Init()) {
    PLOG_ERROR("ApaZmqOutputModule failed to initialize CARLA ZMQ output bridge.");
    return false;
  }
  PLOG_INFO("ApaZmqOutputModule init: zmq_control_enable={}",
            apa_zmq_output_params_.zmq_control_enable());
  return true;
}

bool ApaZmqOutputModule::InitIO() {
  CREATE_INPUT_RETURN_IF_ERROR(ApaZmqOutputModule, apa_output,
                               pangu::modules::ApaOutput);
  PLOG_INFO("ApaZmqOutputModule InitIO end");
  return true;
}

bool ApaZmqOutputModule::LoadConfig(const std::string& ptstr) {
  if (!google::protobuf::TextFormat::ParseFromString(ptstr,
                                                     &apa_zmq_output_params_)) {
    PLOG_ERROR("Can't get conf file of [{}]", module_name_);
    return false;
  }
  return true;
}

bool ApaZmqOutputModule::Proc() { return true; }

DEFINE_CALLBACK_HEAD(ApaZmqOutputModule, apa_output, pangu::modules::ApaOutput) {
  std::string inputer;
  auto msg = GET_TRIGGER_MSG_DATA_PROTO(inputer, pangu::modules::ApaOutput);
  if (msg == nullptr) {
    PLOG_WARN("apa_output callback got null message from {}", inputer);
    return;
  }

  if (!apa_zmq_output_params_.zmq_control_enable()) {
    return;
  }
  if (!zmq_output_bridge_) {
    PLOG_ERROR("ApaZmqOutputModule apa_output callback before ZMQ init.");
    return;
  }
  if (!zmq_output_bridge_->Publish(*msg)) {
    PLOG_WARN("ApaZmqOutputModule failed to publish ZMQ APA output.");
  }
}

}
}
