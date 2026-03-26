#include "UFLD.h"

int main(int argc, char** argv)
{
    cv::VideoCapture inputVideo;
    cv::String video = "../test.avi";
    inputVideo.open(video);
    UFLDLaneDetector laneDetector;
    
    while(inputVideo.grab())
    {
        inputVideo.retrieve(laneDetector.img);
        if(laneDetector.img.empty()) return false;
        laneDetector.run();
        laneDetector.display();
    }
    return 0;
}
