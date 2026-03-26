
/**
 * @brief 卡尔曼滤波
 * @file funcKalmanFilter.cpp
 * @version 0.0.1
 * @author Tianqi Ke (ktq23@mails.tsinghua.edu.cn)
 * @date 2023-12-15
 */
#include "funcKalmanFilter.h"


namespace perception{

/**
* @brief kalmanFilterKALInitiate 函数
* @param[IN] param1, KalmanFilter类的对象
* @param[IN] input1, 初始化参数
* @param[OUT] output1,  均值、协方差

* @cn_name: 卡尔曼滤波初始化

* @granularity: atomic 

* @tag: perception
*/
void kalmanFilterKALInitiate(ParamStruct01 &param1, InputStruct01 &input1, OutputStruct01 &output1)
{
    KalmanFilter &c = param1.c;
    output1.output = c.initiate(input1.measurement);
    return;
}



/**
* @brief kalmanFilterPredict 函数
* @param[IN] param2, KalmanFilter类的对象
* @param[IN] input2,  均值和协方差
* @param[OUT] output2, 空输出

* @cn_name: 卡尔曼滤波预测

* @granularity: atomic 

* @tag: perception
*/
void kalmanFilterPredict(ParamStruct02 &param2, InputStruct02 &input2, OutputStruct02 &output2)
{
    KalmanFilter &c = param2.c;
    c.predict(input2.mean,input2.covariance);
    return;
}





/**
* @brief kalmanFilterUpdate 函数
* @param[IN] param4, KalmanFilter类的对象
* @param[IN] input4, 均值、协方差、测量值
* @param[OUT] output4, 更新结果

* @cn_name: 卡尔曼滤波更新

* @granularity: atomic 

* @tag: perception
*/
void kalmanFilterUpdate(ParamStruct04 &param4, InputStruct04 &input4, OutputStruct04 &output4)
{
    KalmanFilter &c = param4.c;
    output4.output = c.update(input4.mean,input4.covariance,input4.measurement);
    return;
}




}
