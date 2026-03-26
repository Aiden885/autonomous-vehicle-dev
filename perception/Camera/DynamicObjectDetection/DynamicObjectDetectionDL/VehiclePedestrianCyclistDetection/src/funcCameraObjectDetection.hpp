/**
 * @brief 文件说明
 * @file funcCameraObjectDetection.hpp
 * @version 0.0.1
 * @author Chuang Zhang
 * @date 2023-11-30
 */
#pragma once

#include <iostream>
#include <opencv2/core.hpp>
#include "smoke.hh"
using namespace cv;

typedef struct
{
    SMOKE detector;

} ParamStruct01;

typedef struct
{
    string enginePath;
    cv::Mat intrinsic;
} InputStruct01;

typedef struct
{

} OutputStruct01;

typedef struct
{
    SMOKE detector;
} ParamStruct02;

typedef struct
{
    string enginePath;
} InputStruct02;

typedef struct
{

} OutputStruct02;

typedef struct
{
    SMOKE detector;
} ParamStruct03;

typedef struct
{
    cv::Mat intrinsic;
} InputStruct03;

typedef struct
{

} OutputStruct03;

typedef struct
{
    SMOKE detector;
} ParamStruct04;

typedef struct
{
    cv::Mat input;
} InputStruct04;

typedef struct
{

} OutputStruct04;

typedef struct
{
    int INPUT_H;
    int INPUT_W;

} ParamStruct05;

typedef struct
{
    cv::Mat resizeInput;
} InputStruct05;

typedef struct
{
    cv::Mat resizeOutput;
} OutputStruct05;

typedef struct
{
    SMOKE detector;
} ParamStruct06;

typedef struct
{
    cv::Mat colorInput;
} InputStruct06;

typedef struct
{
    cv::Mat colorOutput;

} OutputStruct06;
typedef struct
{
    SMOKE detector;
} ParamStruct07;

typedef struct
{

} InputStruct07;

typedef struct
{

} OutputStruct07;

typedef struct
{
    SMOKE detector;
    int TOPK = 100;
    float SCORE_THRESH = 0.3;
} ParamStruct08;

typedef struct
{
    cv::Mat input;

} InputStruct08;

typedef struct
{

} OutputStruct08;

typedef struct
{
    SMOKE detector;
    float SCORE_THRESH;
} ParamStruct09;

typedef struct
{
    int i;

} InputStruct09;

typedef struct
{
    bool isObject;

} OutputStruct09;

typedef struct
{
    SMOKE detector;
} ParamStruct10;

typedef struct
{
    int i;
    float x, y, z;

} InputStruct10;

typedef struct
{

} OutputStruct10;

typedef struct
{
    SMOKE detector;
} ParamStruct11;

typedef struct
{
    int i;
    float x, y, z, w, l, h;

} InputStruct11;

typedef struct
{

} OutputStruct11;

typedef struct
{
    SMOKE detector;
} ParamStruct12;
typedef struct
{
    int i;
    float x, y, z, angle;

} InputStruct12;

typedef struct
{

} OutputStruct12;

typedef struct
{
    SMOKE detector;
} ParamStruct13;
typedef struct
{
    cv::Mat input;

} InputStruct13;

typedef struct
{

} OutputStruct13;

void cameraDetectorInit(ParamStruct01 &, InputStruct01 &, OutputStruct01 &);
void cameraDetectorEngineInit(ParamStruct02 &, InputStruct02 &, OutputStruct02 &);
void cameraIntrinsicInit(ParamStruct03 &, InputStruct03 &, OutputStruct03 &);
void cameraDetectorPreProcess(ParamStruct04 &, InputStruct04 &, OutputStruct04 &);
void cameraDetectorPreProcessResize(ParamStruct05 &, InputStruct05 &, OutputStruct05 &);
void cameraDetectorPreProcessColor(ParamStruct06 &, InputStruct06 &, OutputStruct06 &);
void cameraDetectorInference(ParamStruct07 &, InputStruct07 &, OutputStruct07 &);
void cameraDetectorPostProcess(ParamStruct08 &, InputStruct08 &, OutputStruct08 &);
void cameraDetectorResultSocre(ParamStruct09 &, InputStruct09 &, OutputStruct09 &);
void cameraDetectorResultPositionTrans(ParamStruct10 &, InputStruct10 &, OutputStruct10 &);
void cameraDetectorResultSizeTrans(ParamStruct11 &, InputStruct11 &, OutputStruct11 &);
void cameraDetectorResultAngle(ParamStruct12 &, InputStruct12 &, OutputStruct12 &);
void cameraDetectorResultCam2Image(ParamStruct13 &, InputStruct13 &, OutputStruct13 &);
