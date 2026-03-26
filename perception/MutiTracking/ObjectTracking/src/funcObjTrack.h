 
#pragma once

/**
 * @brief 多目标跟踪
 * @file funcObjTrack.h
 * @version 0.0.1
 * @author Tianqi Ke (ktq23@mails.tsinghua.edu.cn)
 * @date 2023-12-15
 */
#include "tracker.h"

/**
 * \namespace perception
 * \brief perception namespace
 */

namespace perception{

typedef struct{
    
}ParamStruct01;


typedef struct{
    float max_euclidean_distance = 1.5;
    float max_iou_distance = 0.9;
    int max_age = 15;
    int n_init = 5;
    float dt = 0.1;
}InputStruct01;


typedef struct{
    tracker output;
}OutputStruct01;

/**
* @brief trackerInit 函数
* @param[IN] param1, 空参数
* @param[IN] input1, 初始化参数
* @param[OUT] output1, tracker类的对象

* @cn_name 多目标kalman跟踪初始化

* @granularity atomic 

* @tag perception
*/
void trackerInit(ParamStruct01 &param1, InputStruct01 &input1, OutputStruct01 &output1);
// void _init(float max_euclidean_distance = 1.5, float max_iou_distance = 0.9, int max_age = 15,
//                int n_init = 5, float dt = 0.1,
//                const std::vector<std::string> &classes = {"pedestrian", "vehicle", "cyclist", "others"});




typedef struct{
    tracker c;
}ParamStruct02;


typedef struct{
    
}InputStruct02;


typedef struct{
    
}OutputStruct02;

/**
* @brief trackerPredict 函数
* @param[IN] param2, tracker类的对象
* @param[IN] input2, 空输入
* @param[OUT] output2, 空输出

* @cn_name 多目标kalman跟踪预测

* @granularity atomic 

* @tag perception
*/
void trackerPredict(ParamStruct02 &param2, InputStruct02 &input2, OutputStruct02 &output2);
// void predict();





typedef struct{
    tracker c;
}ParamStruct03;


typedef struct{
    const DETECTIONS detections;
}InputStruct03;


typedef struct{
    
}OutputStruct03;

/**
* @brief trackerUpdate 函数
* @param[IN] param3, tracker类的对象
* @param[IN] input3, 观测值
* @param[OUT] output3, 空输出

* @cn_name 多目标kalman跟踪更新

* @granularity atomic 

* @tag perception
*/
void trackerUpdate(ParamStruct03 &param3, InputStruct03 &input3, OutputStruct03 &output3);
//   void update(const DETECTIONS &detections);
}