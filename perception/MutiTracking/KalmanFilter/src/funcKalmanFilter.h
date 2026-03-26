 
#pragma once


/**
 * @brief 卡尔曼滤波
 * @file funcKalmanFilter.h
 * @version 0.0.1
 * @author Tianqi Ke (ktq23@mails.tsinghua.edu.cn)
 * @date 2023-12-15
 */

#include "kalmanfilter.h"

/**
 * \namespace perception
 * \brief perception namespace
 */

namespace perception{

typedef struct{
    KalmanFilter c;
}ParamStruct01;


typedef struct{
    const DETECTBOX measurement;
}InputStruct01;


typedef struct{
    KAL_DATA output;
}OutputStruct01;

/**
* @brief kalmanFilterKALInitiate 函数
* @param[IN] param1, KalmanFilter类的对象
* @param[IN] input1, 初始化参数
* @param[OUT] output1,  均值、协方差

* @cn_name 卡尔曼滤波初始化

* @granularity atomic 

* @tag perception
*/
void kalmanFilterKALInitiate(ParamStruct01 &param1, InputStruct01 &input1, OutputStruct01 &output1);
// KAL_DATA initiate(const DETECTBOX &measurement);



typedef struct{
    KalmanFilter c;
}ParamStruct02;


typedef struct{
    KAL_MEAN mean;
    KAL_COVA covariance;
}InputStruct02;


typedef struct{
    
}OutputStruct02;

/**
* @brief kalmanFilterPredict 函数
* @param[IN] param2, KalmanFilter类的对象
* @param[IN] input2,  均值和协方差
* @param[OUT] output2, 空输出

* @cn_name 卡尔曼滤波预测

* @granularity atomic 

* @tag perception
*/
void kalmanFilterPredict(ParamStruct02 &param2, InputStruct02 &input2, OutputStruct02 &output2);
// void predict(KAL_MEAN &mean, KAL_COVA &covariance);








typedef struct{
    KalmanFilter c;
}ParamStruct04;


typedef struct{
    const KAL_MEAN mean;
    const KAL_COVA covariance;
    const DETECTBOX measurement;
}InputStruct04;


typedef struct{
    KAL_DATA output;
}OutputStruct04;

/**
* @brief kalmanFilterUpdate 函数
* @param[IN] param4, KalmanFilter类的对象
* @param[IN] input4, 均值、协方差、测量值
* @param[OUT] output4, 更新结果

* @cn_name 卡尔曼滤波更新

* @granularity atomic 

* @tag perception
*/
void kalmanFilterUpdate(ParamStruct04 &param4, InputStruct04 &input4, OutputStruct04 &output4);
// KAL_DATA update(const KAL_MEAN &mean, const KAL_COVA &covariance, const DETECTBOX &measurement);


}