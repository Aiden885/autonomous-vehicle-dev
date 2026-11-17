#pragma once

#include <memory>
#include <string>

#include "common/os_adapter/os_api.h"

#include "apa_config.pb.h"
#include "apa_zmq_bridge.h"

namespace pangu {
namespace modules {

/**
 * @brief APA CARLA ZMQ output bridge module
 * @cn_name APA CARLA ZMQ输出桥接节点
 * @granularity composite
 * @tag demo
 * @version 1.0.0
 */
class ApaZmqOutputModule : public icvos::adapter::IcvosBaseModule {
 public:
  ApaZmqOutputModule() : icvos::adapter::IcvosBaseModule("ApaZmqOutputModule") {}
  ~ApaZmqOutputModule() override = default;

  virtual bool Init(const std::string& ptstr = "") override;
  virtual bool InitIO() override;
  virtual bool Proc() override;
  virtual void SetVersion() override;
  virtual bool LoadConfig(const std::string& ptstr = "") override;
  DECLARE_CALLBACK(apa_output, pangu::modules::ApaOutput);

 private:
  ApaZmqOutputModuleParams apa_zmq_output_params_;
  std::unique_ptr<ApaZmqOutputBridge> zmq_output_bridge_;
  DECLARE_SUBSCRIBER(apa_output);

 private:
  REGISTER_NODE_DECLARE(ApaZmqOutputModule);
};

}
}
