/* 真值表 */
int truth_table_ca11a4a0_e76b_470f_9504_1776d3c041b1(
    short u,  // [in] 真值表输入端口
    short v  // [in] 真值表输入端口
)
{
    int y;
    
    y=1;

    
    y=2;

    
    y=3;

    
    y=4;

    
    y=7;

    
    y=5;

    
    y=5;

    
    y=6;

    if(u==0&&(v==6||v==7)) {
      y=8;
    }
    if(!(u==0&&(v==6||v==7))) {
      y=8;
    }

    
    return y;
}