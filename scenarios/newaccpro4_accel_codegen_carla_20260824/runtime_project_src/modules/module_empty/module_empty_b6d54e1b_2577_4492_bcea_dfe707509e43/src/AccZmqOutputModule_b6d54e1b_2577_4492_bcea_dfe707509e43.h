
#pragma once

#include <cstdint>
#include <memory>

#include <string>

#include "common/os_adapter/os_api.h"

#include "module_empty_b6d54e1b_2577_4492_bcea_dfe707509e43_config.pb.h"
#include "acc_zmq_bridge.h"


namespace pangu {
namespace modules {/**
 * @brief CARLA ZMQ output bridge module
 * @cn_name CARLA ZMQ输出桥接模块
 * @granularity composite
 * @tag demo
 * @version 1.0.0
 */
class AccZmqOutputModule_b6d54e1b_2577_4492_bcea_dfe707509e43 : public icvos::adapter::IcvosBaseModule {
public:
  AccZmqOutputModule_b6d54e1b_2577_4492_bcea_dfe707509e43() : icvos::adapter::IcvosBaseModule("AccZmqOutputModule_b6d54e1b_2577_4492_bcea_dfe707509e43") {}
  virtual ~AccZmqOutputModule_b6d54e1b_2577_4492_bcea_dfe707509e43() { PLOG_DEBUG("=========== in virtual ~AccZmqOutputModule_b6d54e1b_2577_4492_bcea_dfe707509e43"); }

  virtual bool Init(const std::string& ptstr = "") override;
  virtual bool InitIO() override;
  virtual bool Proc() override;
  virtual void SetVersion() override;
  virtual bool LoadConfig(const std::string& ptstr = "") override;

  DECLARE_CALLBACK(acc_output, pangu::modules::AccOutput);

 private:
  AccZmqOutputModule_b6d54e1b_2577_4492_bcea_dfe707509e43Params acc_zmq_output_params_;
  std::unique_ptr<AccZmqOutputBridge> zmq_output_bridge_;

  DECLARE_SUBSCRIBER(acc_output);

 private:
  REGISTER_NODE_DECLARE(AccZmqOutputModule_b6d54e1b_2577_4492_bcea_dfe707509e43);
};


} // namespace pangu
} // namespace modules
