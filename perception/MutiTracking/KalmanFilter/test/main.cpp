/**
 * @brief 测试用例
 * @file main.cpp
 * @version 0.0.1
 * @author Tianqi Ke (ktq23@mails.tsinghua.edu.cn)
 * @date 2023-12-15
 */

#include <iostream>
#include "funcKalmanFilter.h"
#include "kalmanfilter.h"

int main(){   
    // kalmanfilter.h的测试
    std::cout << "kalmanfilter.h test: " << std::endl;
    KalmanFilter kf(0.1);

    // 创建检测数据，比如位置x=10, y=20，速度vx=3, vy=2
    DETECTBOX measurement;
    measurement << 10, 20, 3, 2;

    // 调用initiate函数
    KAL_DATA initState = kf.initiate(measurement);

    // 输出初始状态
    std::cout << "Initial State: " << initState.first << std::endl;

    // 预测下一步
    KAL_MEAN predictedMean;
    KAL_COVA predictedCovariance;
    kf.predict(predictedMean, predictedCovariance);

    // 输出预测状态
    std::cout << "Predicted Mean: " << predictedMean << std::endl;
    std::cout << "Predicted Covariance: " << predictedCovariance << std::endl;

    // 更新状态
    KAL_DATA updatedState = kf.update(predictedMean, predictedCovariance, measurement);

    // 输出更新后的状态
    std::cout << "Updated State: " << updatedState.first << std::endl;


    std::cout<<std::endl;



    // funcKalmanFilter.h的测试
    std::cout << "funcKalmanFilter.h test: " << std::endl;
    KalmanFilter kf2(0.1);
    // 创建检测数据，比如位置x=10, y=20，速度vx=3, vy=2
    DETECTBOX measurement2;
    measurement2 << 10, 20, 3, 2;
    ParamStruct01 param1 = {kf2};
    InputStruct01 input1 = {measurement2};
    OutputStruct01 output1;
    kalmanFilterKALInitiate(param1,input1,output1);
    // 输出初始状态
    std::cout << "Initial State: " << output1.output.first << std::endl;

    // 预测下一步
    KAL_MEAN predictedMean2;
    KAL_COVA predictedCovariance2;
    ParamStruct02 param2 = {kf2};
    InputStruct02 input2 = {predictedMean2,predictedCovariance2};
    OutputStruct02 output2;
    kalmanFilterPredict(param2,input2,output2);
    // 输出预测状态
    std::cout << "Predicted Mean: " << predictedMean2 << std::endl;
    std::cout << "Predicted Covariance: " << predictedCovariance2 << std::endl;




    ParamStruct04 param4 = {kf2};
    InputStruct04 input4 = {predictedMean2, predictedCovariance2, measurement2};
    OutputStruct04 output4; 
    kalmanFilterUpdate(param4,input4,output4);
     // 输出更新后的状态
    std::cout << "Updated State: " << output4.output.first << std::endl;

    return 0;
}
