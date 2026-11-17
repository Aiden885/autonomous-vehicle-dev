
#include "LksZmqInputModule_a807407f_21e6_442d_a6e4_13a788ec666d.h"

#include <google/protobuf/text_format.h>


namespace pangu {
namespace modules {REGISTER_NODE_DEFINE(LksZmqInputModule_a807407f_21e6_442d_a6e4_13a788ec666d);

void LksZmqInputModule_a807407f_21e6_442d_a6e4_13a788ec666d::SetVersion() {
  SETVERSION(module_name_, "1.0.0");
  SHOWVERSION();
}

bool LksZmqInputModule_a807407f_21e6_442d_a6e4_13a788ec666d::Init(const std::string& ptstr) {
  SetVersion();

  if (!LoadConfig(ptstr)) {
    PLOG_ERROR("Can't load conf file of [{}]", module_name_);
    return false;
  }

  if (!InitIO()) {
    PLOG_ERROR("creat io for {} failed, please check!", module_name_);
    return false;
  }

  zmq_input_bridge_ = std::make_unique<LksZmqBridge>();
  if (!zmq_input_bridge_->Init()) {
    PLOG_ERROR("LksZmqInputModule_a807407f_21e6_442d_a6e4_13a788ec666d failed to initialize CARLA ZMQ input bridge.");
    return false;
  }
  frame_id_counter_ = 0;
  CREATE_TIMER_TASK(LksZmqInputModule_a807407f_21e6_442d_a6e4_13a788ec666d, params_.timer_duration(),
                    LksZmqInputModule_a807407f_21e6_442d_a6e4_13a788ec666d::Proc);
  PLOG_INFO("LksZmqInputModule_a807407f_21e6_442d_a6e4_13a788ec666d init: timer_duration={}, "
            "zmq_poll_timeout_ms={}",
            params_.timer_duration(),
            params_.zmq_poll_timeout_ms());
  return true;
}

bool LksZmqInputModule_a807407f_21e6_442d_a6e4_13a788ec666d::InitIO() {
  CREATE_OUTPUT_RETURN_IF_ERROR(lks_input, pangu::modules::LksInput);

  PLOG_INFO("LksZmqInputModule_a807407f_21e6_442d_a6e4_13a788ec666d InitIO end");
  return true;
}

bool LksZmqInputModule_a807407f_21e6_442d_a6e4_13a788ec666d::LoadConfig(const std::string& ptstr) {
  if (!google::protobuf::TextFormat::ParseFromString(ptstr,
                                                     &params_)) {
    PLOG_ERROR("Can't get conf file of [{}]", module_name_);
    return false;
  }

  return true;
}

bool LksZmqInputModule_a807407f_21e6_442d_a6e4_13a788ec666d::Proc() {
  if (!zmq_input_bridge_) {
    PLOG_ERROR("LksZmqInputModule_a807407f_21e6_442d_a6e4_13a788ec666d Proc called before initialization.");
    return false;
  }

  if (!zmq_input_bridge_->Poll(params_.zmq_poll_timeout_ms())) {
    PLOG_WARN("LksZmqInputModule_a807407f_21e6_442d_a6e4_13a788ec666d failed to poll CARLA ZMQ bridge.");
    return false;
  }

  if (!zmq_input_bridge_->HasFreshInput()) {
    return true;
  }

  const auto zmq_state = zmq_input_bridge_->GetState();
  if (!zmq_state.ego_valid || !zmq_state.lane_valid) {
    return true;
  }

  auto input = std::make_shared<pangu::modules::LksInput>();
  input->set_frame_id(frame_id_counter_++);
  input->set_ego_speed_mps(zmq_state.ego_speed_mps);
  input->set_brake_pressed(zmq_state.brake_pressed);
  input->set_driver_steer_norm(zmq_state.driver_steer_norm);
  input->set_c0_m(zmq_state.c0_m);
  input->set_c1(zmq_state.c1);
  input->set_c2_per_m(zmq_state.c2_per_m);
  input->set_c3_per_m2(zmq_state.c3_per_m2);
  input->set_curvature_per_m(zmq_state.curvature_per_m);
  PUB_MSG(lks_input, input);
  return true;
}


} // namespace pangu
} // namespace modules
