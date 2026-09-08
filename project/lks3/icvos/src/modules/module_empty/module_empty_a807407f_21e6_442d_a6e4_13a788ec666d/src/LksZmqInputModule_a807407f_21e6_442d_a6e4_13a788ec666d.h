
#pragma once

#include <cstdint>
#include <memory>

#include <string>

#include "common/os_adapter/os_api.h"

#include "module_empty_a807407f_21e6_442d_a6e4_13a788ec666d_config.pb.h"
#include "lks_zmq_bridge.h"


namespace pangu {
namespace modules {/**
 * @brief CARLA ZMQ input bridge module
 * @cn_name CARLA ZMQ输入桥接模块
 * @granularity composite
 * @tag demo
 * @version 1.0.0
 */
class LksZmqInputModule_a807407f_21e6_442d_a6e4_13a788ec666d : public icvos::adapter::IcvosBaseModule {
public:
  LksZmqInputModule_a807407f_21e6_442d_a6e4_13a788ec666d() : icvos::adapter::IcvosBaseModule("LksZmqInputModule_a807407f_21e6_442d_a6e4_13a788ec666d") {}
  virtual ~LksZmqInputModule_a807407f_21e6_442d_a6e4_13a788ec666d() { PLOG_DEBUG("=========== in virtual ~LksZmqInputModule_a807407f_21e6_442d_a6e4_13a788ec666d"); }

  virtual bool Init(const std::string& ptstr = "") override;
  virtual bool InitIO() override;
  virtual bool Proc() override;
  virtual void SetVersion() override;
  virtual bool LoadConfig(const std::string& ptstr = "") override;

 private:
  LksZmqInputModule_a807407f_21e6_442d_a6e4_13a788ec666dParams params_;
  std::unique_ptr<LksZmqInputBridge> zmq_input_bridge_;
  uint32_t frame_id_counter_ = 0;

 private:
  REGISTER_NODE_DECLARE(LksZmqInputModule_a807407f_21e6_442d_a6e4_13a788ec666d);
};


} // namespace pangu
} // namespace modules