#pragma once

#include <memory>
#include <string>

#include "common/os_adapter/os_api.h"

#include "module_empty_d3fdb632_0c78_41ab_a837_3bc336dba2df_config.pb.h"
#include "acc_zmq_bridge.h"


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
  uint32_t frame_id_counter_ = 0;

 private:
  REGISTER_NODE_DECLARE(ZmqBridgeModule);
};


} // namespace pangu
} // namespace modules