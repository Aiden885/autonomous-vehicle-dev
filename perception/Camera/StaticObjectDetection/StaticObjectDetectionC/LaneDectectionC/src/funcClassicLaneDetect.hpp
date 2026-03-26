/**
 * @brief 文件说明 
 * @file funcClassicLaneDetect.hpp
 * @version 0.0.1
 * @author Chi Ding (dc22@mails.tsinghua.edu.com)
 * @date 2023-12-28
 */

#include <opencv2/opencv.hpp>
#include <opencv2/highgui.hpp>
#include <vector>
#include "ClassicLaneDetect.hpp"

typedef struct{

}ParamStruct01;

typedef struct{
    ClassicLaneDetect laneDetect;
}InputStruct01;

typedef struct{

}OutputStruct01;


void classicLaneDetectRun (ParamStruct01 &,InputStruct01 &,OutputStruct01 &);



typedef struct{
    vector<Point> chain;
    int n;
    Mat a;
}ParamStruct02;

typedef struct{
    ClassicLaneDetect laneDetect;
}InputStruct02;

typedef struct{

}OutputStruct02;

void classicLaneDetectPolyFit (ParamStruct02 &,InputStruct02 &,OutputStruct02 &);



typedef struct{
    Mat originImage;
}ParamStruct03;


typedef struct{
    ClassicLaneDetect laneDetect;
}InputStruct03;

typedef struct{

}OutputStruct03;


void classicLaneDetectGetPerspectiveMatrix (ParamStruct03 &,InputStruct03 &,OutputStruct03 &);



typedef struct{
    Mat originImage;
}ParamStruct04;

typedef struct{
    ClassicLaneDetect laneDetect;
}InputStruct04;

typedef struct{
     Mat compensateImage;
}OutputStruct04;

void classicLaneDetectLightCompensate (ParamStruct04 &,InputStruct04 &,OutputStruct04 &);



typedef struct{
    Mat originImage;
}ParamStruct05;

typedef struct{
    ClassicLaneDetect laneDetect;
}InputStruct05;

typedef struct{
     Mat enhancedImage;
}OutputStruct05;

void classicLaneDetectImageEnhancement (ParamStruct05 &,InputStruct05 &,OutputStruct05 &);




typedef struct{
    Mat originImage;
}ParamStruct06;


typedef struct{
    ClassicLaneDetect laneDetect;
}InputStruct06;


typedef struct{
     Mat processImage;
}OutputStruct06;

void classicLaneDetectPreProcess (ParamStruct06 &,InputStruct06 &,OutputStruct06 &);




typedef struct{
    Mat image;
    vector<Point> leftPoints1;
    vector<Point> leftPoints2;
    vector<Point> rightPoints1;
    vector<Point> rightPoints2;
}ParamStruct07;


typedef struct{
    ClassicLaneDetect laneDetect;
}InputStruct07;


typedef struct{

}OutputStruct07;

void classicLaneDetectDetectLaneByEdges (ParamStruct07 &,InputStruct07 &,OutputStruct07 &);




typedef struct{
    vector<Point> leftPoints1;
    vector<Point> leftPoints2;
    vector<Point> rightPoints1;
    vector<Point> rightPoints2;
}ParamStruct08;


typedef struct{
    ClassicLaneDetect laneDetect;
}InputStruct08;


typedef struct{
    Mat leftFunc;
    Mat rightFunc;
}OutputStruct08;

void classicLaneDetectGenerateFinalLanes (ParamStruct08 &,InputStruct08 &,OutputStruct08 &);




typedef struct{
    Mat showImage;
    Mat laneFunc;
}ParamStruct09;


typedef struct{
    ClassicLaneDetect laneDetect;
}InputStruct09;


typedef struct{
   
}OutputStruct09;

void classicLaneDetectDrawLane (ParamStruct09 &,InputStruct09 &,OutputStruct09 &);




typedef struct{
  
}ParamStruct10;


typedef struct{
    ClassicLaneDetect laneDetect;
}InputStruct10;


typedef struct{
   
}OutputStruct10;

void classicLaneDetectDisplayLanes (ParamStruct10 &,InputStruct10 &,OutputStruct10 &);




typedef struct{
  
}ParamStruct11;


typedef struct{
    ClassicLaneDetect laneDetect;
}InputStruct11;


typedef struct{
   
}OutputStruct11;

void classicLaneDetectModuleSelfCheck (ParamStruct11 &,InputStruct11 &,OutputStruct11 &);




typedef struct{
  
}ParamStruct12;


typedef struct{
    ClassicLaneDetect laneDetect;
}InputStruct12;


typedef struct{
   
}OutputStruct12;

void classicLaneDetectModuleSelfCheck (ParamStruct12 &,InputStruct12 &,OutputStruct12 &);




typedef struct{
    string path;
    vector<string> imageNames;
}ParamStruct13;


typedef struct{
  
}InputStruct13;


typedef struct{
   
}OutputStruct13;

void classicLaneDetectGetImagesName (ParamStruct13 &,InputStruct13 &,OutputStruct13 &);