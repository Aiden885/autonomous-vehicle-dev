
#pragma once

#include <cstdint>
#include <memory>

#include <string>

#include "common/os_adapter/os_api.h"

#include "module_empty_b6d54e1b_2577_4492_bcea_dfe707509e43_config.pb.h"
#include "acc_zmq_bridge.h"


namespace pangu {
namespace modules {/**
 * @brief CARLA ZMQ input bridge module
 * @cn_name CARLA ZMQ输入桥接模块
 * @granularity composite
 * @tag demo
 * @version 1.0.0
 */
class AccZmqInputModule_b6d54e1b_2577_4492_bcea_dfe707509e43 : public icvos::adapter::IcvosBaseModule {
public:
  AccZmqInputModule_b6d54e1b_2577_4492_bcea_dfe707509e43() : icvos::adapter::IcvosBaseModule("AccZmqInputModule_b6d54e1b_2577_4492_bcea_dfe707509e43") {}
  virtual ~AccZmqInputModule_b6d54e1b_2577_4492_bcea_dfe707509e43() { PLOG_DEBUG("=========== in virtual ~AccZmqInputModule_b6d54e1b_2577_4492_bcea_dfe707509e43"); }

  virtual bool Init(const std::string& ptstr = "") override;
  virtual bool InitIO() override;
  virtual bool Proc() override;
  virtual void SetVersion() override;
  virtual bool LoadConfig(const std::string& ptstr = "") override;

 private:
  AccZmqInputModule_b6d54e1b_2577_4492_bcea_dfe707509e43Params acc_zmq_input_params_;
  std::unique_ptr<AccZmqInputBridge> zmq_input_bridge_;
  uint32_t frame_id_counter_ = 0;

  DECLARE_PUBLISHER(acc_input, pangu::modules::AccInput);

 private:
  REGISTER_NODE_DECLARE(AccZmqInputModule_b6d54e1b_2577_4492_bcea_dfe707509e43);
};


} // namespace pangu
} // namespace modules
