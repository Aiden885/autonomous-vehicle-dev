
#pragma once

#include <string>

#include "common/os_adapter/os_api.h"  // 是调用系统的接口头文件，必须添加

#include "module_empty_b6d54e1b_2577_4492_bcea_dfe707509e43_config.pb.h"

#include "run_f1807d24_61ef_407b_932e_4df61bda48e7.hpp"


namespace pangu {
namespace modules {/**
 * @brief 空模块
 * @cn_name module empty
 * @granularity composite
 * @tag common
 * @author 
 * @version 1.0.0
*/
class module_empty_b6d54e1b_2577_4492_bcea_dfe707509e43 : public icvos::adapter::IcvosBaseModule {
 public:
  module_empty_b6d54e1b_2577_4492_bcea_dfe707509e43() : icvos::adapter::IcvosBaseModule("module_empty_b6d54e1b_2577_4492_bcea_dfe707509e43") {}
  virtual ~module_empty_b6d54e1b_2577_4492_bcea_dfe707509e43() { PLOG_DEBUG("=========== in virtual ~module_empty_b6d54e1b_2577_4492_bcea_dfe707509e43"); }

  virtual bool Init(const std::string &ptstr) override;

  virtual bool InitIO() override;

  virtual bool Proc() override;

  virtual void SetVersion() override;

  virtual bool LoadConfig(const std::string& ptstr = "") override;

  /*声明订阅或者定时器回调的callback任务*/
  DECLARE_CALLBACK(acc_input, pangu::modules::AccInput);
  control::run_f1807d24_61ef_407b_932e_4df61bda48e7 composite_block_instance_{control::run_f1807d24_61ef_407b_932e_4df61bda48e7Traits::Param{5, 0.25, 0.4}, control::run_f1807d24_61ef_407b_932e_4df61bda48e7Traits::State{}};
  void acc_inputCallback(const pangu::modules::AccInput& input);

 private:
  module_empty_b6d54e1b_2577_4492_bcea_dfe707509e43Params module_empty_b6d54e1b_2577_4492_bcea_dfe707509e43_params_;
  DECLARE_PUBLISHER(acc_output, pangu::modules::AccOutput);
  
  DECLARE_SUBSCRIBER(acc_input);
  

 private:
  REGISTER_NODE_DECLARE(module_empty_b6d54e1b_2577_4492_bcea_dfe707509e43);
};


} // namespace pangu
} // namespace modules