#pragma once

#include <memory>
#include <string>

#include "common/os_adapter/os_api.h"

#include "ACCModule_config.pb.h"
#include "AccTargetSpeed.hpp"

namespace pangu {
namespace modules {

/**
 * @brief ACC reference module
 * @cn_name ACC参考模块
 * @granularity composite
 * @tag demo
 * @version 2.0.0
 */
class ACCModule : public icvos::adapter::IcvosBaseModule {
public:
  ACCModule() : icvos::adapter::IcvosBaseModule("ACCModule") {}
  virtual ~ACCModule() { PLOG_DEBUG("=========== in virtual ~ACCModule"); }

  virtual bool Init(const std::string& ptstr = "") override;
  virtual bool InitIO() override;
  virtual bool Proc() override;
  virtual void SetVersion() override;
  virtual bool LoadConfig(const std::string& ptstr = "") override;

  DECLARE_CALLBACK(acc_input, pangu::modules::AccInput);

private:
  void acc_inputCallback(const pangu::modules::AccInput& input);

  control::AccTargetSpeed acc_target_speed_;

  DECLARE_PUBLISHER(acc_output, pangu::modules::AccOutput);
  DECLARE_SUBSCRIBER(acc_input);

private:
  REGISTER_NODE_DECLARE(ACCModule);
};

}  // namespace modules
}  // namespace pangu
