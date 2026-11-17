#include "truth_table_8d98fa7f_4daf_49ae_bb29_69662b8ce656.hpp"

namespace control {


/* 真值表组件，用于处理复杂判断场景 */
int truth_table_8d98fa7f_4daf_49ae_bb29_69662b8ce656(
    short systemState,  // [in] 真值表输入端口
    short commandType  // [in] 真值表输入端口
)
{
    int decisionCode;
    if((systemState==0&&commandType==1)) {
      decisionCode=1;
    }
    else if((systemState==0&&commandType==2)) {
      decisionCode=2;
    }
    else if((systemState==0&&commandType==3)) {
      decisionCode=3;
    }
    else if((systemState==0&&commandType==4)) {
      decisionCode=4;
    }
    else if((systemState==0&&commandType==5)) {
      decisionCode=7;
    }
    else if((systemState==1&&commandType==1)) {
      decisionCode=5;
    }
    else if((systemState==2&&commandType==1)) {
      decisionCode=5;
    }
    else if((systemState==1&&commandType==2)) {
      decisionCode=6;
    }
    else if((systemState==0&&(commandType==6||commandType==7))) {
      decisionCode=8;
    }
    else if(!(systemState==0&&commandType==1) && !(systemState==0&&commandType==2) && !(systemState==0&&commandType==3) && !(systemState==0&&commandType==4) && !(systemState==0&&commandType==5) && !(systemState==1&&commandType==1) && !(systemState==2&&commandType==1) && !(systemState==1&&commandType==2) && !(systemState==0&&(commandType==6||commandType==7))) {
      decisionCode=0;
    }

    
    return decisionCode;
}

} // namespace control
