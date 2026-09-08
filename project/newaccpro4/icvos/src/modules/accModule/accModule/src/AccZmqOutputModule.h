
#pragma once

#include <cstdint>
#include <memory>

#include <string>

#include "common/os_adapter/os_api.h"

#include "accModule_config.pb.h"
#include "acc_zmq_bridge.h"


namespace pangu {
namespace modules {/**
 * @brief CARLA ZMQ output bridge module
 * @cn_name CARLA ZMQ输出桥接模块
 * @granularity composite
 * @tag demo
 * @version 1.0.0
 */
class AccZmqOutputModule : public icvos::adapter::IcvosBaseModule {
public:
  AccZmqOutputModule() : icvos::adapter::IcvosBaseModule("AccZmqOutputModule") {}
  virtual ~AccZmqOutputModule() { PLOG_DEBUG("=========== in virtual ~AccZmqOutputModule"); }

  virtual bool Init(const std::string& ptstr = "") override;
  virtual bool InitIO() override;
  virtual bool Proc() override;
  virtual void SetVersion() override;
  virtual bool LoadConfig(const std::string& ptstr = "") override;
  DECLARE_CALLBACK(acc_output, pangu::modules::AccOutput);

 private:
  AccZmqOutputModuleParams acc_zmq_output_params_;
  std::unique_ptr<AccZmqOutputBridge> zmq_output_bridge_;
  DECLARE_SUBSCRIBER(acc_output);

 private:
  REGISTER_NODE_DECLARE(AccZmqOutputModule);
};


} // namespace pangu
} // namespace modules