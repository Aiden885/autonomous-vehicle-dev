/**
 * @brief 测试用例
 * @file main.cpp
 * @version 0.0.1
 * @author Tianqi Ke (ktq23@mails.tsinghua.edu.cn)
 * @date 2023-12-15
 */

#include <iostream>
#include "funcMatching.h"


int main(){   
    // 创建 NearNeighborDisMetric 实例，选择欧几里得距离作为度量类型
    NearNeighborDisMetric metric1(NearNeighborDisMetric::euclidean, 0.5f);

    // 创建一些测试数据
    POSITION dets_pos(2, 2); // 假设是 2x2 矩阵
    POSITION tracks_pos(2, 2); // 假设也是 2x2 矩阵

    // 填充一些随机或预定的值
    dets_pos << 1.0, 2.0,
                3.0, 4.0;
    tracks_pos << 5.0, 6.0,
                  7.0, 8.0;

    // 调用 distance 方法
    DYNAMICM result1 = metric1.distance(dets_pos, tracks_pos);
   

    // 打印结果
    std::cout << "Distance: \n" << result1 << std::endl;
   

    NearNeighborDisMetric metric2(NearNeighborDisMetric::cosine, 0.5f);
    DYNAMICM result2=metric2.distance(dets_pos, tracks_pos);
    std::cout << "Distance: \n" << result2 << std::endl;

    ParamStruct01 th1 = {0.5f};
    InputStruct01 input1 = {dets_pos,tracks_pos};
    OutputStruct01 output1;
    nearNeighborCosineDistance(th1,input1,output1);
    std::cout << "Distance: \n" << output1.output << std::endl;


    ParamStruct02 th2 = {0.5f};
    InputStruct02 input2 = {dets_pos,tracks_pos};
    OutputStruct02 output2;
    nearNeighborEuclideanDistance(th2,input2,output2);
    std::cout << "Distance: \n" << output2.output << std::endl;
    




    return 0;
}
