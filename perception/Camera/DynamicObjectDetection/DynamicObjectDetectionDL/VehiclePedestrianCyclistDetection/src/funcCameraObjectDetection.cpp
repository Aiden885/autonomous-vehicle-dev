/**
 * @brief 基于相机的目标检测
 * @file funcCameraObjectDetection.cpp
 * @version 0.0.1
 * @author Chuang Zhang
 * @date 2023-11-30
 */
#include "funcCameraObjectDetection.hpp"

/**
 * @brief 检测器初始化
 * @param[IN] param 检测器类
 * @param[IN] input 无
 * @param[OUT] output 无
 
 * @cn_name: 检测器初始化
 
 * @granularity: composite //函数组件的粒度， atomic： 基础组件（原子），composite： 复合组件（在前端可以向下一层展开）
 
 * @tag: percepetion
 */
void cameraDetectorInit(ParamStruct01 &param01, InputStruct01 &input01, OutputStruct01 &output01)
{
    SMOKE &smokeDetector = param01.detector;
    ParamStruct02 param02{.detector = smokeDetector};
    InputStruct02 input02{.enginePath = input01.enginePath};
    OutputStruct02 output02;
    cameraDetectorEngineInit(param02, input02, output02);
    ParamStruct03 param03{.detector = smokeDetector};
    InputStruct03 input03{.intrinsic = input01.intrinsic};
    OutputStruct03 output03;
    cameraIntrinsicInit(param03, input03, output03);
}
/**
 * @brief 检测引擎初始化
 * @param[IN] param 检测器类
 * @param[IN] input 检测引擎路径
 * @param[OUT] output 无
 
 * @cn_name: 检测引擎初始化
 
 * @granularity: atomic //函数组件的粒度， atomic： 基础组件（原子），composite： 复合组件（在前端可以向下一层展开）
 
 * @tag: percepetion
 */
void cameraDetectorEngineInit(ParamStruct02 &param02, InputStruct02 &input02, OutputStruct02 &output02)
{
    SMOKE &smokeDetector = param02.detector;
    smokeDetector.loadEngine(input02.enginePath);
}
/**
 * @brief 摄像头内参初始化
 * @param[IN] param 检测器类
 * @param[IN] input 摄像头内参
 * @param[OUT] output 无
 
 * @cn_name: 摄像头内参初始化
 
 * @granularity: atomic //函数组件的粒度， atomic： 基础组件（原子），composite： 复合组件（在前端可以向下一层展开）
 
 * @tag: percepetion
 */
void cameraIntrinsicInit(ParamStruct03 &param03, InputStruct03 &input03, OutputStruct03 &output03)
{
    SMOKE &smokeDetector = param03.detector;
    smokeDetector.intrinsicInit(input03.intrinsic);
}
/**
 * @brief 检测器图像预处理
 * @param[IN] param 检测器类
 * @param[IN] input 输入图像
 * @param[OUT] output 无
 
 * @cn_name: 检测器图像预处理
 
 * @granularity: composite //函数组件的粒度， atomic： 基础组件（原子），composite： 复合组件（在前端可以向下一层展开）
 
 * @tag: percepetion
 */
void cameraDetectorPreProcess(ParamStruct04 &param04, InputStruct04 &input04, OutputStruct04 &output04)
{
    SMOKE &smokeDetector = param04.detector;
    ParamStruct05 param05{.INPUT_H = 384, .INPUT_W = 1280};
    InputStruct05 input05{.resizeInput = input04.input};
    OutputStruct05 output05{.resizeOutput = input04.input};
    cameraDetectorPreProcessResize(param05, input05, output05);
    ParamStruct06 param06{.detector = smokeDetector};
    InputStruct06 input06{.colorInput = output05.resizeOutput};
    OutputStruct06 output06{.colorOutput = output05.resizeOutput};
    cameraDetectorPreProcessColor(param06, input06, output06);
}
/**
 * @brief 检测器图像尺寸变换
 * @param[IN] param 尺寸变换参数
 * @param[IN] input 尺寸变换输入图像
 * @param[OUT] output 尺寸变换输出图像
 
 * @cn_name: 检测器图像尺寸变换
 
 * @granularity: atomic //函数组件的粒度， atomic： 基础组件（原子），composite： 复合组件（在前端可以向下一层展开）
 
 * @tag: percepetion
 */
void cameraDetectorPreProcessResize(ParamStruct05 &param05, InputStruct05 &input05, OutputStruct05 &output05)
{
    cv::resize(input05.resizeInput, output05.resizeOutput, cv::Size(param05.INPUT_W, param05.INPUT_H), cv::INTER_LINEAR);
}
/**
 * @brief 检测器图像色彩变换
 * @param[IN] param 检测器类
 * @param[IN] input 色彩变换输入图像
 * @param[OUT] output 色彩变换输出图像
 
 * @cn_name: 检测器图像色彩变换
 
 * @granularity: atomic //函数组件的粒度， atomic： 基础组件（原子），composite： 复合组件（在前端可以向下一层展开）
 
 * @tag: percepetion
 */
void cameraDetectorPreProcessColor(ParamStruct06 &param06, InputStruct06 &input06, OutputStruct06 &output06)
{
    SMOKE &smokeDetector = param06.detector;
    smokeDetector.preProcess(param06.colorInput);
    param06.colorOutput = param06.colorInput;
}
/**
 * @brief 检测器推理
 * @param[IN] param 检测器类
 * @param[IN] input 无
 * @param[OUT] output 无
 
 * @cn_name: 检测器推理
 
 * @granularity: atomic //函数组件的粒度， atomic： 基础组件（原子），composite： 复合组件（在前端可以向下一层展开）
 
 * @tag: percepetion
 */
void cameraDetectorInference(ParamStruct07 &param07, InputStruct07 &input07, OutputStruct07 &output07)
{
    SMOKE &smokeDetector = param07.detector;
    smokeDetector.inference();
}

/**
 * @brief 检测器后处理
 * @param[IN] param 检测器类，检测结果最大数量，检测结果阈值
 * @param[IN] input 图像
 * @param[OUT] output 无
 
 * @cn_name: 检测器后处理
 
 * @granularity: composite //函数组件的粒度， atomic： 基础组件（原子），composite： 复合组件（在前端可以向下一层展开）
 
 * @tag: percepetion
 */
void cameraDetectorPostProcess(ParamStruct08 &param08, InputStruct08 &input08, OutputStruct08 &output08)
{
    SMOKE &smokeDetector = param08.detector;
    for (int i = 0; i < param08.TOPK; ++i)
    {
        PramaStruct09 param09{.detector = smokeDetector, .SCORE_THRESH = param08.SCORE_THRESH};
        InputStruct09 input09{.i = i};
        OutputStruct09 output09{.isObject = false};
        cameraDetectorResultSocre(param09, input09, output09);
        if (output09.isObject == false)
        {
            continue;
        }
        float x, y, z, w, l, h, angle = 0.0;
        ParamStruct10 param10{.detector = smokeDetector};
        InputStruct10 input10{.i = i, .x = x, .y = y, .z = z};
        OutputStruct10 output10;
        cameraDetectorResultPositionTrans(param10, input10, output10);

        ParamStruct11 param11{.detector = smokeDetector};
        InputStruct11 input11{.i = i, .x = x, .y = y, .z = z, .w = w, .l = l, .h = h};
        OutputStruct11 output11;
        cameraDetectorResultSizeTrans(param11, input11, output11);

        ParamStruct12 param12{.detector = smokeDetector};
        InputStruct12 input12{.i = i, .x = x, .y = y, .z = z, .angle = angle};
        OutputStruct12 output12;
        cameraDetectorResultAngle(param12, input12, output12);

        ParamStruct13 param13{.detector = smokeDetector};
        InputStruct13 input13{.input = input08.input};
        OutputStruct13 output13;
        cameraDetectorResultCam2Image(param13, input13, output13);
    }
}
/**
 * @brief 检测结果筛选
 * @param[IN] param 检测器类，检测结果阈值
 * @param[IN] input 检测结果索引
 * @param[OUT] output 是否为目标
 
 * @cn_name: 检测结果筛选
 
 * @granularity: atomic //函数组件的粒度， atomic： 基础组件（原子），composite： 复合组件（在前端可以向下一层展开）
 
 * @tag: percepetion
 */
void cameraDetectorResultSocre(ParamStruct09 &param09, InputStruct08 &input09, OutputStruct08 &output09)
{
    SMOKE &smokeDetector = param09.detector;
    if (smokeDetector.topkScores[input09.i] < param09.SCORE_THRESH)
    {
        output09.isObject = false;
    }
    else
    {
        output09.isObject = true;
    }
}
/**
 * @brief 检测结果位置变换
 * @param[IN] param 检测器类
 * @param[IN] input 检测结果索引，位置变换中心
 * @param[OUT] output 无

 * @cn_name: 检测结果位置变换

 * @granularity: atomic //函数组件的粒度， atomic： 基础组件（原子），composite： 复合组件（在前端可以向下一层展开）

 * @tag: percepetion
 */
void cameraDetectorResultPositionTrans(ParamStruct10 &param10, InputStruct10 &input10, OutputStruct10 &output10)
{
    SMOKE &smokeDetector = param10.detector;
    smokeDetector.bBoxPositionTrans(input09.i, input09.x, input09.y, input09.z);
}
/**
 * @brief 检测结果尺寸变换
 * @param[IN] param 检测器类
 * @param[IN] input 检测结果索引，尺寸变换参数（中心及缩放比例）
 * @param[OUT] output 无
 
 * @cn_name: 检测结果尺寸变换
 
 * @granularity: atomic //函数组件的粒度， atomic： 基础组件（原子），composite： 复合组件（在前端可以向下一层展开）
 
 * @tag: percepetion
 */
void cameraDetectorResultSizeTrans(ParamStruct11 &param11, InputStruct11 &input11, OutputStruct11 &output11)
{
    SMOKE &smokeDetector = param11.detector;
    smokeDetector.bBoxSizeTrans(input11.i, input11.x, input11.y, input11.z, input11.w, input11.l, input11.h);
}
/**
 * @brief 检测结果角度变换
 * @param[IN] param 检测器类
 * @param[IN] input 检测结果索引，角度变换参数（中心及角度）
 * @param[OUT] output 无
 
 * @cn_name: 检测结果角度变换
 
 * @granularity: atomic //函数组件的粒度， atomic： 基础组件（原子），composite： 复合组件（在前端可以向下一层展开）
 
 * @tag: percepetion
 */
void cameraDetectorResultAngle(ParamStruct12 &param12, InputStruct12 &input12, OutputStruct12 &output12)
{
    SMOKE &smokeDetector = param12.detector;
    smokeDetector.bBoxAngle(input12.i, input12.x, input12.y, input12.z, input12.angle);
}
/**
 * @brief 检测结果相机坐标系转图像坐标系
 * @param[IN] param 检测器类
 * @param[IN] input 输入图像
 * @param[OUT] output 无
 
 * @cn_name: 检测结果相机坐标系转图像坐标系
 
 * @granularity: atomic //函数组件的粒度， atomic： 基础组件（原子），composite： 复合组件（在前端可以向下一层展开）
 
 * @tag: percepetion
 */
void cameraDetectorResultCam2Image(ParamStruct13 &param13, InputStruct13 &input13, OutputStruct13 &output13)
{
    SMOKE &smokeDetector = param13.detector;
    smokeDetector.bBoxCam2Image(input13.input);
}
