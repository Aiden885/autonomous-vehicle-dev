/**
 * @brief 文件说明 
 * @file funcUFLD.hpp
 * @version 0.0.1
 * @author Chi Ding (dc22@mails.tsinghua.edu.com)
 * @date 2024-01-05
 */

#include <opencv2/opencv.hpp>
#include <opencv2/highgui.hpp>
#include <vector>
#include "UFLD.h"



typedef struct{
    string configFile;
}ParamStruct01;


typedef struct{
    UFLDLaneDetector UFLDlaneDetect;
}InputStruct01;


typedef struct{

}OutputStruct01;

void funcUFLDLaneDetectReadConfig (ParamStruct01 &,InputStruct01 &,OutputStruct01 &);



typedef struct{
    string engineFile;
}ParamStruct02;

typedef struct{
    UFLDLaneDetector UFLDlaneDetect;
}InputStruct02;

typedef struct{

}OutputStruct02;

void funcUFLDLaneDetectInitTRTmodel (ParamStruct02 &,InputStruct02 &,OutputStruct02 &);




typedef struct{
    string calibFile;
}ParamStruct03;


typedef struct{
    UFLDLaneDetector UFLDlaneDetect;
}InputStruct03;


typedef struct{

}OutputStruct03;

void funcUFLDLaneDetectInitCalib (ParamStruct03 &,InputStruct03 &,OutputStruct03 &);



typedef struct{
    Mat img;
    vector<float> processedImg;
}ParamStruct04;


typedef struct{
    UFLDLaneDetector UFLDlaneDetect;
}InputStruct04;


typedef struct{

}OutputStruct04;

void funcUFLDLaneDetectPreProcess (ParamStruct04 &,InputStruct04 &,OutputStruct04 &);



typedef struct{
   IExecutionContext &context;
   float input;
   float output;
   int batchSize;
}ParamStruct05;


typedef struct{
    UFLDLaneDetector UFLDlaneDetect;
}InputStruct05;


typedef struct{

}OutputStruct05;

void funcUFLDLaneDetectDoInference (ParamStruct05 &,InputStruct05 &,OutputStruct05 &);



typedef struct{
    float x;
    float y;
    int rows;
    int cols;
    int chan;
}ParamStruct06;

typedef struct{
    UFLDLaneDetector UFLDlaneDetect;
}InputStruct06;


typedef struct{

}OutputStruct06;

void funcUFLDLaneDetectSoftmax_mul (ParamStruct06 &,InputStruct06 &,OutputStruct06 &);



typedef struct{
    float x;
    float y;
    int rows;
    int cols;
    int chan;
}ParamStruct07;


typedef struct{
    UFLDLaneDetector UFLDlaneDetect;
}InputStruct07;

typedef struct{

}OutputStruct07;

void funcUFLDLaneDetectArgmax (ParamStruct07 &,InputStruct07 &,OutputStruct07 &);




typedef struct{

}ParamStruct08;


typedef struct{
    UFLDLaneDetector UFLDlaneDetect;
}InputStruct08;


typedef struct{

}OutputStruct08;

void funcUFLDLaneDetectPostProcess (ParamStruct08 &,InputStruct08 &,OutputStruct08 &);



typedef struct{

}ParamStruct09;

typedef struct{
    UFLDLaneDetector UFLDlaneDetect;
}InputStruct09;


typedef struct{
    bool outputlane;
}OutputStruct09;

void funcUFLDLaneDetectDisplay (ParamStruct09 &,InputStruct09 &,OutputStruct09 &);




typedef struct{

}ParamStruct10;


typedef struct{
    UFLDLaneDetector UFLDlaneDetect;
}InputStruct10;


typedef struct{

}OutputStruct10;

void funcUFLDLaneDetectToProto (ParamStruct10 &,InputStruct10 &,OutputStruct10 &);




typedef struct{

}ParamStruct11;


typedef struct{
    UFLDLaneDetector UFLDlaneDetect;
}InputStruct11;


typedef struct{

}OutputStruct11;

void funcUFLDLaneDetectRun (ParamStruct11 &,InputStruct11 &,OutputStruct11 &);