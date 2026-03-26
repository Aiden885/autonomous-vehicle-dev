/**
 * @brief 多目标跟踪
 * @file funcObjTrack.cpp
 * @version 0.0.1
 * @author Tianqi Ke (ktq23@mails.tsinghua.edu.cn)
 * @date 2023-12-15
 */
#include "funcObjTrack.h"

/**
 * \namespace perception
 * \brief perception namespace
 */

namespace perception{

/**
* @brief trackerInit 函数
* @param[IN] param1, 空参数
* @param[IN] input1, 初始化参数
* @param[OUT] output1, tracker类的对象

* @cn_name: 多目标kalman跟踪初始化

* @granularity: atomic 

* @tag: perception
*/
void trackerInit(ParamStruct01 &param1, InputStruct01 &input1, OutputStruct01 &output1)
{
    output1.output._init(input1.max_euclidean_distance,input1.max_iou_distance,input1.max_age,input1.n_init,input1.dt);
    return;
}
// void _init(float max_euclidean_distance = 1.5, float max_iou_distance = 0.9, int max_age = 15,
//                int n_init = 5, float dt = 0.1,
//                const std::vector<std::string> &classes = {"pedestrian", "vehicle", "cyclist", "others"});




/**
* @brief trackerPredict 函数
* @param[IN] param2, tracker类的对象
* @param[IN] input2, 空输入
* @param[OUT] output2, 空输出

* @cn_name: 多目标kalman跟踪预测

* @granularity: atomic 

* @tag: perception
*/
void trackerPredict(ParamStruct02 &param2, InputStruct02 &input2, OutputStruct02 &output2)
{
    param2.c.predict();
    return;
}
// void predict();


/**
* @brief trackerUpdate 函数
* @param[IN] param3, tracker类的对象
* @param[IN] input3, 观测值
* @param[OUT] output3, 空输出

* @cn_name: 多目标kalman跟踪更新

* @granularity: atomic 

* @tag: perception
*/
void trackerUpdate(ParamStruct03 &param3, InputStruct03 &input3, OutputStruct03 &output3)
{
    param3.c.update(input3.detections);
    return;
}





}