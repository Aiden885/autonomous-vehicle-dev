
#pragma once

#include <memory>
#include <string>

#include "common/os_adapter/os_api.h"

#include "ACCModule_config.pb.h"
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
  DECLARE_CALLBACK(acc_output, pangu::modules::AccOutput);

 private:
  ZmqBridgeModuleParams zmq_bridge_params_;
  std::unique_ptr<AccZmqBridge> zmq_bridge_;
  uint32_t frame_id_counter_ = 0;
  DECLARE_PUBLISHER(acc_input, pangu::modules::AccInput);
  DECLARE_SUBSCRIBER(acc_output);

 private:
  REGISTER_NODE_DECLARE(ZmqBridgeModule);
};


} // namespace pangu
} // namespace modules