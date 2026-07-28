
#pragma once

#include <string>

#include "common/os_adapter/os_api.h"  // 是调用系统的接口头文件，必须添加

#include "module_empty_d3fdb632_0c78_41ab_a837_3bc336dba2df_config.pb.h"



namespace pangu {
namespace modules {/**
 * @brief 空模块
 * @cn_name module empty
 * @granularity composite
 * @tag common
 * @author 
 * @version 1.0.0
*/
class module_empty_d3fdb632_0c78_41ab_a837_3bc336dba2df : public icvos::adapter::IcvosBaseModule {
 public:
  module_empty_d3fdb632_0c78_41ab_a837_3bc336dba2df() : icvos::adapter::IcvosBaseModule("module_empty_d3fdb632_0c78_41ab_a837_3bc336dba2df") {}
  virtual ~module_empty_d3fdb632_0c78_41ab_a837_3bc336dba2df() { PLOG_DEBUG("=========== in virtual ~module_empty_d3fdb632_0c78_41ab_a837_3bc336dba2df"); }

  virtual bool Init(const std::string &ptstr) override;

  virtual bool InitIO() override;

  virtual bool Proc() override;

  virtual void SetVersion() override;

  virtual bool LoadConfig(const std::string& ptstr = "") override;

  /*声明订阅或者定时器回调的callback任务*/

 private:
  module_empty_d3fdb632_0c78_41ab_a837_3bc336dba2dfParams module_empty_d3fdb632_0c78_41ab_a837_3bc336dba2df_params_;

 private:
  REGISTER_NODE_DECLARE(module_empty_d3fdb632_0c78_41ab_a837_3bc336dba2df);
};


} // namespace pangu
} // namespace modules