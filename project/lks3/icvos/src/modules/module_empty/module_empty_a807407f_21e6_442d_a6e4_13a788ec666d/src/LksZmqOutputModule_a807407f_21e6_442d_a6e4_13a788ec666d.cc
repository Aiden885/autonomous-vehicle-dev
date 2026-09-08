
#include "LksZmqOutputModule_a807407f_21e6_442d_a6e4_13a788ec666d.h"

#include <google/protobuf/text_format.h>


namespace pangu {
namespace modules {REGISTER_NODE_DEFINE(LksZmqOutputModule_a807407f_21e6_442d_a6e4_13a788ec666d);

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

  zmq_output_bridge_ = std::make_unique<LksZmqOutputBridge>();
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


} // namespace pangu
} // namespace modules