/* 真值表组件，用于处理复杂判断场景 */
int truth_table_22a78862_36c9_4204_a31a_a7baf5ca8592(
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