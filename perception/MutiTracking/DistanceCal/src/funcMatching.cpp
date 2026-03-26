
/**
 * @brief 匹配功能
 * @file funcMatching.cpp
 * @version 0.0.1
 * @author Tianqi Ke (ktq23@mails.tsinghua.edu.cn)
 * @date 2023-12-15
 */
#include "funcMatching.h"


namespace perception{
/**
* @brief nearNeighborCosineDistance 函数
* @param[IN] param1,匹配阈值
* @param[IN] input1, 位置点
* @param[OUT] output1, Eigen::Matrix<float, -1, -1, Eigen::RowMajor>

* @cn_name: 计算余弦距离

* @granularity: atomic 

* @tag: perception
*/
void nearNeighborCosineDistance(ParamStruct01 &param1, InputStruct01 &input1, OutputStruct01 &output1)
{
    NearNeighborDisMetric c(NearNeighborDisMetric::cosine, param1.matching_threshold);
    output1.output = c.distance(input1.dets_pos,input1.tracks_pos);
    return;
}



/**
* @brief nearNeighborEuclideanDistance 函数
* @param[IN] param2, 匹配阈值
* @param[IN] input2, 位置点
* @param[OUT] output2, Eigen::Matrix<float, -1, -1, Eigen::RowMajor>

* @cn_name: 计算欧拉距离

* @granularity: atomic 

* @tag: perception
*/
void nearNeighborEuclideanDistance(ParamStruct02 &param2, InputStruct02 &input2, OutputStruct02 &output2)
{
    NearNeighborDisMetric c(NearNeighborDisMetric::euclidean, param2.matching_threshold);
    output2.output = c.distance(input2.dets_pos,input2.tracks_pos);
    return;
}
}