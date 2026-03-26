 
#pragma once


#include "nn_matching.h"

/**
 * \namespace perception
 * \brief perception namespace
 */

namespace perception{

/**
 * @brief 匹配功能
 * @file funcMatching.h
 * @version 0.0.1
 * @author Tianqi Ke (ktq23@mails.tsinghua.edu.cn)
 * @date 2023-12-15
 */


typedef struct{
    float matching_threshold;
}ParamStruct01;


typedef struct{
    const POSITION dets_pos;
    const POSITION tracks_pos;
}InputStruct01;


typedef struct{
    DYNAMICM output;
}OutputStruct01;

/**
* @brief nearNeighborCosineDistance 函数
* @param[IN] param1,匹配阈值
* @param[IN] input1, 位置点
* @param[OUT] output1, Eigen::Matrix<float, -1, -1, Eigen::RowMajor>

* @cn_name 计算余弦距离

* @granularity atomic 

* @tag perception
*/
void nearNeighborCosineDistance(ParamStruct01 &param1, InputStruct01 &input1, OutputStruct01 &output1);
// DYNAMICM _nncosine_distance(const POSITION &dets_pos, const POSITION &tracks_pos);



typedef struct{
    float matching_threshold;
}ParamStruct02;


typedef struct{
    const POSITION dets_pos;
    const POSITION tracks_pos;
}InputStruct02;


typedef struct{
    DYNAMICM output;
}OutputStruct02;

/**
* @brief nearNeighborEuclideanDistance 函数
* @param[IN] param2, 匹配阈值
* @param[IN] input2, 位置点
* @param[OUT] output2, Eigen::Matrix<float, -1, -1, Eigen::RowMajor>

* @cn_name 计算欧拉距离

* @granularity atomic 

* @tag perception
*/
void nearNeighborEuclideanDistance(ParamStruct02 &param2, InputStruct02 &input2, OutputStruct02 &output2);
// DYNAMICM _nneuclidean_distance(const POSITION &dets_pos, const POSITION &tracks_pos);


}