
#include "module_empty.h"
#include <google/protobuf/text_format.h>


namespace pangu {
namespace modules {/*节点注册*/
REGISTER_NODE_DEFINE(module_empty);

void module_empty::SetVersion() {
  SETVERSION(module_name_, "1.0.0");
  SETVERSION("class1", "1.0.1");
}

bool module_empty::Init(const std::string &ptstr) {

  SetVersion();
  if (!LoadConfig(ptstr)) {
    PLOG_ERROR("can't load proto str for module:{}", module_name_);
    return false;
  }
  if (!InitIO()) {
    PLOG_ERROR("can't create io for module:{}", module_name_);
    return false;
  }

  return true;
}
bool module_empty::InitIO() {
  PLOG_INFO("in  InitIO");

  return true;
}

bool module_empty::LoadConfig(const std::string &ptstr) {
  return true;
}

bool module_empty::Proc() {
  return true;
}


} // namespace pangu
} // namespace modules