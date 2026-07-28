#include "ZmqBridgeModule.h"

#include <google/protobuf/text_format.h>


namespace pangu {
namespace modules {REGISTER_NODE_DEFINE(ZmqBridgeModule);

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
  return true;
}


} // namespace pangu
} // namespace modules