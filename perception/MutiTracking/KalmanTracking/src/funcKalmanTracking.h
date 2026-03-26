 
#pragma once
/**
 * @brief 卡尔曼跟踪
 * @file funcKalmanTracking.h
 * @version 0.0.1
 * @author Tianqi Ke (ktq23@mails.tsinghua.edu.cn)
 * @date 2023-12-15
 */

#include "track.h"

/**
 * \namespace perception
 * \brief perception namespace
 */

namespace perception{

typedef struct{
    KalmanFilter c;
}ParamStruct01;


typedef struct{
    KAL_MEAN mean;
    KAL_COVA covariance;
}InputStruct01;


typedef struct{
    
}OutputStruct01;


void kalmanFilterPredict(ParamStruct01 &param1, InputStruct01 &input1, OutputStruct01 &output1);



typedef struct{
    Track c;
}ParamStruct02;


typedef struct{
    std::shared_ptr<KalmanFilter> kf;
}InputStruct02;


typedef struct{
    
}OutputStruct02;

/**
* @brief kalmanTrackPredict 函数
* @param[IN] param2, Track类的对象
* @param[IN] input2, KalmanFilter类的智能指针
* @param[OUT] output2, 空输出

* @cn_name 卡尔曼跟踪预测

* @granularity atomic

* @tag perception
*/
void kalmanTrackPredict(ParamStruct02 &param2, InputStruct02 &input2, OutputStruct02 &output2);
//  void predit(std::shared_ptr<KalmanFilter> &kf);





typedef struct{
    KalmanFilter c;
}ParamStruct03;


typedef struct{
    const KAL_MEAN mean;
    const KAL_COVA covariance;
    const DETECTBOX measurement;
}InputStruct03;


typedef struct{
    KAL_DATA output;
}OutputStruct03;


void kalmanFilterUpdate(ParamStruct03 &param3, InputStruct03 &input3, OutputStruct03 &output3);





typedef struct{
    Track c;
}ParamStruct04;


typedef struct{
    std::shared_ptr<KalmanFilter> kf;
    const DETECTION_ROW detection;
}InputStruct04;


typedef struct{
   
}OutputStruct04;

/**
* @brief kalmanTrackUpdate 函数
* @param[IN] param4, Track类的对象
* @param[IN] input4, KalmanFilter类的智能指针、观测值
* @param[OUT] output4, 空输出

* @cn_name 卡尔曼跟踪更新

* @granularity atomic 

* @tag perception
*/
void kalmanTrackUpdate(ParamStruct04 &param4, InputStruct04 &input4, OutputStruct04 &output4);
// void update(std::shared_ptr<KalmanFilter> &kf, const DETECTION_ROW &detection);




typedef struct{
    Track c;
}ParamStruct05;


typedef struct{

}InputStruct05;


typedef struct{
   
}OutputStruct05;

/**
* @brief kalmanTrackMarkMiss 函数
* @param[IN] param5, Track类的对象
* @param[IN] input5, 空输入
* @param[OUT] output5, 空输出

* @cn_name 判断目标是否处于丢失状态

* @granularity atomic 

* @tag perception
*/
void kalmanTrackMarkMiss(ParamStruct05 &param5, InputStruct05 &input5, OutputStruct05 &output5);
// void mark_missed();







typedef struct{
    Track c;
}ParamStruct07;


typedef struct{
    
}InputStruct07;


typedef struct{
   bool is_deleted;
}OutputStruct07;

/**
* @brief kalmanTrackIsDetected 函数
* @param[IN] param7, Track类的对象
* @param[IN] input7, 空输入
* @param[OUT] output7, bool值

* @cn_name 判断目标是否被删除

* @granularity atomic 

* @tag perception
*/
void kalmanTrackIsDetected(ParamStruct07 &param7, InputStruct07 &input7, OutputStruct07 &output7);
// bool is_deleted();



}