
#pragma once

#include <string>

#include "common/os_adapter/os_api.h"  // 是调用系统的接口头文件，必须添加

#include "ACCModule_config.pb.h"



namespace pangu {
namespace modules {/**
 * @brief ACC reference module
 * @cn_name module empty
 * @granularity composite
 * @tag common
 * @author 
 * @version 1.0.0
*/
class ACCModule : public icvos::adapter::IcvosBaseModule {
 public:
  ACCModule() : icvos::adapter::IcvosBaseModule("ACCModule") {}
  virtual ~ACCModule() { PLOG_DEBUG("=========== in virtual ~ACCModule"); }

  virtual bool Init(const std::string &ptstr) override;

  virtual bool InitIO() override;

  virtual bool Proc() override;

  virtual void SetVersion() override;

  virtual bool LoadConfig(const std::string& ptstr = "") override;

  /*声明订阅或者定时器回调的callback任务*/
  DECLARE_CALLBACK(acc_input, pangu::modules::AccInput);
  void acc_inputCallback(const pangu::modules::AccInput& input);

 private:
  ACCModuleParams ACCModule_params_;
  DECLARE_PUBLISHER(acc_output, pangu::modules::AccOutput);
  
  DECLARE_SUBSCRIBER(acc_input);
  

 private:
  REGISTER_NODE_DECLARE(ACCModule);
};


} // namespace pangu
} // namespace modules