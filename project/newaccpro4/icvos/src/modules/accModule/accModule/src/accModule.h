
#pragma once

#include <string>

#include "common/os_adapter/os_api.h"  // 是调用系统的接口头文件，必须添加

#include "accModule_config.pb.h"

#include "run_23d2a150_83e4_470e_a9b7_189357a9dfd1.hpp"


namespace pangu {
namespace modules {/**
 * @brief 空模块
 * @cn_name module empty
 * @granularity composite
 * @tag common
 * @author 
 * @version 1.0.0
*/
class accModule : public icvos::adapter::IcvosBaseModule {
 public:
  accModule() : icvos::adapter::IcvosBaseModule("accModule") {}
  virtual ~accModule() { PLOG_DEBUG("=========== in virtual ~accModule"); }

  virtual bool Init(const std::string &ptstr) override;

  virtual bool InitIO() override;

  virtual bool Proc() override;

  virtual void SetVersion() override;

  virtual bool LoadConfig(const std::string& ptstr = "") override;

  /*声明订阅或者定时器回调的callback任务*/
  DECLARE_CALLBACK(acc_input, pangu::modules::AccInput);
  control::run_23d2a150_83e4_470e_a9b7_189357a9dfd1 composite_block_instance_1_{control::run_23d2a150_83e4_470e_a9b7_189357a9dfd1Traits::Param{5, 0.25, 0.4}, control::run_23d2a150_83e4_470e_a9b7_189357a9dfd1Traits::State{}};
  void acc_inputCallback(const pangu::modules::AccInput& input);

 private:
  accModuleParams accModule_params_;
  DECLARE_PUBLISHER(acc_output, pangu::modules::AccOutput);
  
  DECLARE_SUBSCRIBER(acc_input);
  

 private:
  REGISTER_NODE_DECLARE(accModule);
};


} // namespace pangu
} // namespace modules