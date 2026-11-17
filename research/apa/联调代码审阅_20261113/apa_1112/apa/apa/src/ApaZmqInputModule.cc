#include "ApaZmqInputModule.h"

#include <google/protobuf/text_format.h>

namespace pangu {
namespace modules {

REGISTER_NODE_DEFINE(ApaZmqInputModule);

void ApaZmqInputModule::SetVersion() {
  SETVERSION(module_name_, "1.0.0");
  SHOWVERSION();
}

bool ApaZmqInputModule::Init(const std::string& ptstr) {
  SetVersion();
  if (!LoadConfig(ptstr)) {
    PLOG_ERROR("Can't load conf file of [{}]", module_name_);
    return false;
  }
  if (!InitIO()) {
    PLOG_ERROR("creat io for {} failed, please check!", module_name_);
    return false;
  }

  zmq_input_bridge_ = std::make_unique<ApaZmqInputBridge>();
  if (!zmq_input_bridge_->Init()) {
    PLOG_ERROR("ApaZmqInputModule failed to initialize CARLA ZMQ input bridge.");
    return false;
  }

  CREATE_TIMER_TASK(ApaZmqInputModule, apa_zmq_input_params_.timer_duration(),
                    ApaZmqInputModule::Proc);
  PLOG_INFO("ApaZmqInputModule init: timer_duration={}, zmq_poll_timeout_ms={}",
            apa_zmq_input_params_.timer_duration(),
            apa_zmq_input_params_.zmq_poll_timeout_ms());
  return true;
}

bool ApaZmqInputModule::InitIO() {
  CREATE_OUTPUT_RETURN_IF_ERROR(apa_input, pangu::modules::ApaInput);
  PLOG_INFO("ApaZmqInputModule InitIO end");
  return true;
}

bool ApaZmqInputModule::LoadConfig(const std::string& ptstr) {
  if (!google::protobuf::TextFormat::ParseFromString(ptstr,
                                                     &apa_zmq_input_params_)) {
    PLOG_ERROR("Can't get conf file of [{}]", module_name_);
    return false;
  }
  return true;
}

bool ApaZmqInputModule::Proc() {
  std::unique_lock<std::mutex> lock(poll_mutex_, std::try_to_lock);
  if (!lock.owns_lock()) return true;
  if (!zmq_input_bridge_) {
    PLOG_ERROR("ApaZmqInputModule Proc called before initialization.");
    return false;
  }
  if (!zmq_input_bridge_->Poll(apa_zmq_input_params_.zmq_poll_timeout_ms())) {
    PLOG_WARN("ApaZmqInputModule failed to poll CARLA ZMQ bridge.");
    return false;
  }
  if (!zmq_input_bridge_->HasFreshInput()) {
    return true;
  }

  auto input = std::make_shared<pangu::modules::ApaInput>(zmq_input_bridge_->GetInput());
  PUB_MSG(apa_input, input);
  return true;
}

}
}
