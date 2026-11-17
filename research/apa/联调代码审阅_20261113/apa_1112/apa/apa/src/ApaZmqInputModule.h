#pragma once

#include <cstdint>
#include <memory>
#include <mutex>
#include <string>

#include "common/os_adapter/os_api.h"

#include "apa_config.pb.h"
#include "apa_zmq_bridge.h"

namespace pangu {
namespace modules {

/**
 * @brief APA CARLA ZMQ input bridge module
 * @cn_name APA CARLA ZMQ输入桥接节点
 * @granularity composite
 * @tag demo
 * @version 1.0.0
 */
class ApaZmqInputModule : public icvos::adapter::IcvosBaseModule {
 public:
  ApaZmqInputModule() : icvos::adapter::IcvosBaseModule("ApaZmqInputModule") {}
  ~ApaZmqInputModule() override = default;

  virtual bool Init(const std::string& ptstr = "") override;
  virtual bool InitIO() override;
  virtual bool Proc() override;
  virtual void SetVersion() override;
  virtual bool LoadConfig(const std::string& ptstr = "") override;

 private:
  ApaZmqInputModuleParams apa_zmq_input_params_;
  std::unique_ptr<ApaZmqInputBridge> zmq_input_bridge_;
  std::mutex poll_mutex_;
  DECLARE_PUBLISHER(apa_input, pangu::modules::ApaInput);

 private:
  REGISTER_NODE_DECLARE(ApaZmqInputModule);
};

}
}
