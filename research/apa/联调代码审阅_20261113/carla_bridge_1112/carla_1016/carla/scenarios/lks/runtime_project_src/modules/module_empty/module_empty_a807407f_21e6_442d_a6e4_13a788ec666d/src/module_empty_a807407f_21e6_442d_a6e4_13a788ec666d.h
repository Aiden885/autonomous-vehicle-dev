
#pragma once

#include <string>

#include "common/os_adapter/os_api.h"  // 是调用系统的接口头文件，必须添加

#include "module_empty_a807407f_21e6_442d_a6e4_13a788ec666d_config.pb.h"

#include "run_9554bb10_0872_4665_b691_ff372271aaf9.hpp"


namespace pangu {
namespace modules {/**
 * @brief 空模块
 * @cn_name module empty
 * @granularity composite
 * @tag common
 * @author 
 * @version 1.0.0
*/
class module_empty_a807407f_21e6_442d_a6e4_13a788ec666d : public icvos::adapter::IcvosBaseModule {
 public:
  module_empty_a807407f_21e6_442d_a6e4_13a788ec666d() : icvos::adapter::IcvosBaseModule("module_empty_a807407f_21e6_442d_a6e4_13a788ec666d") {}
  virtual ~module_empty_a807407f_21e6_442d_a6e4_13a788ec666d() { PLOG_DEBUG("=========== in virtual ~module_empty_a807407f_21e6_442d_a6e4_13a788ec666d"); }

  virtual bool Init(const std::string &ptstr) override;

  virtual bool InitIO() override;

  virtual bool Proc() override;

  virtual void SetVersion() override;

  virtual bool LoadConfig(const std::string& ptstr = "") override;

  /*声明订阅或者定时器回调的callback任务*/
  DECLARE_CALLBACK(lks_input, pangu::modules::LksInput);
  control::run_9554bb10_0872_4665_b691_ff372271aaf9 execute_lks_{control::run_9554bb10_0872_4665_b691_ff372271aaf9Traits::Param{}, control::run_9554bb10_0872_4665_b691_ff372271aaf9Traits::State{}};
  void lks_inputCallback(const pangu::modules::LksInput& input);

 private:
  module_empty_a807407f_21e6_442d_a6e4_13a788ec666dParams module_empty_a807407f_21e6_442d_a6e4_13a788ec666d_params_;
  DECLARE_PUBLISHER(lks_output, pangu::modules::LksOutput);
  
  DECLARE_SUBSCRIBER(lks_input);
  

 private:
  REGISTER_NODE_DECLARE(module_empty_a807407f_21e6_442d_a6e4_13a788ec666d);
};


} // namespace pangu
} // namespace modules