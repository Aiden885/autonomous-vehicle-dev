//车道线检测器基类
#ifndef _CAMERA_BASE_LANE_
#define _CAMERA_BASE_LANE_
#include <iostream>
#include <opencv2/opencv.hpp>
#include "/home/mht/code/MsgProto/ImgMsg/imgMsg.pb.h"
#include "/home/mht/code/MsgProto/LaneMsg/laneMsg.pb.h"
#include <zmq.h>

class CameraBaseLaneDetector
{
public:
    cv::Mat input;
    lane::LaneList laneResult;
    CameraBaseLaneDetector() {}
    ~CameraBaseLaneDetector() {}
    virtual void run() = 0;
    virtual void receiveImage(cv::Mat inputMat)
    {
        this->input = inputMat;
    }
};
#endif