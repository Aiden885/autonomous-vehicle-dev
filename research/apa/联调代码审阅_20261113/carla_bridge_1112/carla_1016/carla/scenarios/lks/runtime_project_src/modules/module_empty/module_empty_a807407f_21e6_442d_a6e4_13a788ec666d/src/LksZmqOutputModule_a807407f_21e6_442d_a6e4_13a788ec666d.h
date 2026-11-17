
#pragma once

#include <cstdint>
#include <memory>

#include <mutex>

#include <string>

#include "common/os_adapter/os_api.h"

#include "module_empty_a807407f_21e6_442d_a6e4_13a788ec666d_config.pb.h"
#include "lks_zmq_bridge.h"


namespace pangu {
namespace modules {/**
 * @brief CARLA ZMQ output bridge module
 * @cn_name CARLA ZMQ输出桥接模块
 * @granularity composite
 * @tag demo
 * @version 1.0.0
 */
class LksZmqOutputModule_a807407f_21e6_442d_a6e4_13a788ec666d : public icvos::adapter::IcvosBaseModule {
public:
  LksZmqOutputModule_a807407f_21e6_442d_a6e4_13a788ec666d() : icvos::adapter::IcvosBaseModule("LksZmqOutputModule_a807407f_21e6_442d_a6e4_13a788ec666d") {}
  virtual ~LksZmqOutputModule_a807407f_21e6_442d_a6e4_13a788ec666d() { PLOG_DEBUG("=========== in virtual ~LksZmqOutputModule_a807407f_21e6_442d_a6e4_13a788ec666d"); }

  virtual bool Init(const std::string& ptstr = "") override;
  virtual bool InitIO() override;
  virtual bool Proc() override;
  virtual void SetVersion() override;
  virtual bool LoadConfig(const std::string& ptstr = "") override;

  DECLARE_CALLBACK(lks_output, pangu::modules::LksOutput);

 private:
  LksZmqOutputModule_a807407f_21e6_442d_a6e4_13a788ec666dParams params_;
  std::unique_ptr<LksZmqBridge> zmq_output_bridge_;

  DECLARE_SUBSCRIBER(lks_output);

 private:
  REGISTER_NODE_DECLARE(LksZmqOutputModule_a807407f_21e6_442d_a6e4_13a788ec666d);
};


} // namespace pangu
} // namespace modules
