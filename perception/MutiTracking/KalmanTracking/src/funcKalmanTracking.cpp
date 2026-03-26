
/**
 * @brief 卡尔曼跟踪
 * @file funcKalmanTracking.cpp
 * @version 0.0.1
 * @author Tianqi Ke (ktq23@mails.tsinghua.edu.cn)
 * @date 2023-12-15
 */
#include "funcKalmanTracking.h"

namespace perception{

void kalmanFilterPredict(ParamStruct01 &param1, InputStruct01 &input1, OutputStruct01 &output1)
{
    KalmanFilter &c = param1.c;
    c.predict(input1.mean,input1.covariance);
    return;
}


/**
* @brief kalmanTrackPredict 函数
* @param[IN] param2, Track类的对象
* @param[IN] input2, KalmanFilter类的智能指针
* @param[OUT] output2, 空输出

* @cn_name: 卡尔曼跟踪预测

* @granularity: atomic

* @tag: perception
*/
void kalmanTrackPredict(ParamStruct02 &param2, InputStruct02 &input2, OutputStruct02 &output2)
{
    
    KalmanFilter& kf = *input2.kf; // 获取引用，不创建新对象
    ParamStruct01 param1 = {kf};
    Track &c = param2.c;
    InputStruct01 input1 = {c.mean,c.covariance};
    OutputStruct01 output1;
    kalmanFilterPredict(param1,input1,output1);
    c.age += 1;
    c.time_since_update += 1;
    return;
}





void kalmanFilterUpdate(ParamStruct03 &param3, InputStruct03 &input3, OutputStruct03 &output3)
{
    KalmanFilter &c = param3.c;
    output3.output = c.update(input3.mean,input3.covariance,input3.measurement);
    return;
}



/**
* @brief kalmanTrackUpdate 函数
* @param[IN] param4, Track类的对象
* @param[IN] input4, KalmanFilter类的智能指针、观测值
* @param[OUT] output4, 空输出

* @cn_name: 卡尔曼跟踪更新

* @granularity: atomic 

* @tag: perception
*/
void kalmanTrackUpdate(ParamStruct04 &param4, InputStruct04 &input4, OutputStruct04 &output4)
{
    Track &c = param4.c;
    KalmanFilter& kf = *input4.kf; // 获取引用，不创建新对象
    ParamStruct03 param3 = {kf};
    InputStruct03 input3 = {c.mean,c.covariance,input4.detection.xywh};
    OutputStruct03 output3;
    kalmanFilterUpdate(param3,input3,output3);
    KAL_DATA pa = output3.output;
   

    c.mean = pa.first;
    c.covariance = pa.second;

    c.hits += 1;
    c.time_since_update = 0;
    if (c.state == Track::Tentative && c.hits >= c._n_init) 
    {
        c.state = Track::Confirmed; 
    }
    return;
}



/**
* @brief kalmanTrackMarkMiss 函数
* @param[IN] param5, Track类的对象
* @param[IN] input5, 空输入
* @param[OUT] output5, 空输出

* @cn_name: 判断目标是否处于丢失状态

* @granularity: atomic 

* @tag: perception
*/
void kalmanTrackMarkMiss(ParamStruct05 &param5, InputStruct05 &input5, OutputStruct05 &output5)
{
    Track &c = param5.c;
    c.mark_missed();
    return;
}




/**
* @brief kalmanTrackIsDetected 函数
* @param[IN] param7, Track/**
 * \namespace perception
 * \brief perception namespace
 */
类的对象
* @param[IN] input7, 空输入
* @param[OUT] output7, bool值

* @cn_name: 判断目标是否被删除

* @granularity: atomic 

* @tag: perception
*/
void kalmanTrackIsDetected(ParamStruct07 &param7, InputStruct07 &input7, OutputStruct07 &output7)
{
    Track &c = param7.c;
    output7.is_deleted=c.is_deleted();
    return;
}



}