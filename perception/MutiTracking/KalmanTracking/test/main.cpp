/**
 * @brief 测试用例
 * @file main.cpp
 * @version 0.0.1
 * @author Tianqi Ke (ktq23@mails.tsinghua.edu.cn)
 * @date 2023-12-15
 */

#include <iostream>
#include "funcKalmanTracking.h"
#include "kalmanfilter.h"

int main(){   
    KAL_MEAN mean; // 初始化 mean
    KAL_COVA covariance; // 初始化 covariance
    int track_id = 1; // 假设的 track_id
    int n_init = 3; // 假设的 n_init
    int max_age = 5; // 假设的 max_age
    std::string det_class = "class_name"; // 假设的 det_class
    Track track(mean, covariance, track_id, n_init, max_age, det_class);
    std::shared_ptr<KalmanFilter> kf = std::make_shared<KalmanFilter>();
    track.predit(kf); // 检查 track 的 mean 和 covariance 是否更新
    DETECTION_ROW detection; // 初始化 detection
    track.update(kf, detection);// 检查 track 的状态是否更新
    track.mark_missed();
    assert(track.is_deleted()); // 检查轨迹是否被标记为删除
    DETECTBOX tlwh = track.to_tlwh();// 检查转换后的值是否正确
    
    ParamStruct02 param2 = {track};
    InputStruct02 input2 = {kf};
    OutputStruct02 output2;
    kalmanTrackPredict(param2,input2,output2);

    ParamStruct04 param4 = {track};
    InputStruct04 input4 = {kf,detection};
    OutputStruct04 output4;
    kalmanTrackUpdate(param4,input4,output4);



    return 0;
}
