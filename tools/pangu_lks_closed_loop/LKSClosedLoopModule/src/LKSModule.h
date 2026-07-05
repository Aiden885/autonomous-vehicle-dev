#pragma once

#include <string>

#include "common/os_adapter/os_api.h"
#include "LKSModule_config.pb.h"
#include "run_6d9ae1e1_3798_4640_91c6_8217a06e02aa.hpp"

namespace pangu {
namespace modules {

class LKSModule : public icvos::adapter::IcvosBaseModule {
public:
  LKSModule() : icvos::adapter::IcvosBaseModule("LKSModule") {}
  virtual ~LKSModule() = default;

  virtual bool Init(const std::string& ptstr = "") override;
  virtual bool InitIO() override;
  virtual bool Proc() override;
  virtual void SetVersion() override;
  virtual bool LoadConfig(const std::string& ptstr = "") override;

  DECLARE_CALLBACK(lks_input, pangu::modules::LksInput);

private:
  void lks_inputCallback(const pangu::modules::LksInput& input);

  control::run_6d9ae1e1_3798_4640_91c6_8217a06e02aa controller_;
  DECLARE_PUBLISHER(lks_output, pangu::modules::LksOutput);
  DECLARE_SUBSCRIBER(lks_input);
  REGISTER_NODE_DECLARE(LKSModule);
};

}  // namespace modules
}  // namespace pangu
