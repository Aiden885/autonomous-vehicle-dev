#pragma once

#include <memory>
#include <string>

#include "common/os_adapter/os_api.h"
#include "LKSModule_config.pb.h"
#include "lks_zmq_bridge.h"

namespace pangu {
namespace modules {

class LksZmqBridgeModule : public icvos::adapter::IcvosBaseModule {
public:
  LksZmqBridgeModule() : icvos::adapter::IcvosBaseModule("LksZmqBridgeModule") {}
  virtual ~LksZmqBridgeModule() = default;

  virtual bool Init(const std::string& ptstr = "") override;
  virtual bool InitIO() override;
  virtual bool Proc() override;
  virtual void SetVersion() override;
  virtual bool LoadConfig(const std::string& ptstr = "") override;

  DECLARE_CALLBACK(lks_output, pangu::modules::LksOutput);

private:
  LksZmqBridgeModuleParams params_;
  std::unique_ptr<LksZmqBridge> bridge_;
  uint32_t frame_id_ = 0;

  DECLARE_PUBLISHER(lks_input, pangu::modules::LksInput);
  DECLARE_SUBSCRIBER(lks_output);
  REGISTER_NODE_DECLARE(LksZmqBridgeModule);
};

}  // namespace modules
}  // namespace pangu
