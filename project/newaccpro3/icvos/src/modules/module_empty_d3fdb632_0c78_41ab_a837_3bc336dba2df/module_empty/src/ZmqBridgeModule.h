
#pragma once

#include <memory>
#include <string>

#include "common/os_adapter/os_api.h"

#include "module_empty_config.pb.h"
#include "acc_zmq_bridge.h"
#include "composite_block_empty_ea2170e7_297c_4433_b70a_9c66e48988ad.hpp"


namespace pangu {
namespace modules {/**
 * @brief ZMQ bridge companion module
 * @cn_name ZMQ桥接伴生模块
 * @granularity composite
 * @tag demo
 * @version 1.0.0
 */
class ZmqBridgeModule : public icvos::adapter::IcvosBaseModule {
public:
  ZmqBridgeModule() : icvos::adapter::IcvosBaseModule("ZmqBridgeModule") {}
  virtual ~ZmqBridgeModule() { PLOG_DEBUG("=========== in virtual ~ZmqBridgeModule"); }

  virtual bool Init(const std::string& ptstr = "") override;
  virtual bool InitIO() override;
  virtual bool Proc() override;
  virtual void SetVersion() override;
  virtual bool LoadConfig(const std::string& ptstr = "") override;

 private:
  ZmqBridgeModuleParams zmq_bridge_params_;
  std::unique_ptr<AccZmqBridge> zmq_bridge_;
  control::composite_block_empty_ea2170e7_297c_4433_b70a_9c66e48988ad acc_decision_;
  uint32_t frame_id_counter_ = 0;

 private:
  REGISTER_NODE_DECLARE(ZmqBridgeModule);
};


} // namespace pangu
} // namespace modules
