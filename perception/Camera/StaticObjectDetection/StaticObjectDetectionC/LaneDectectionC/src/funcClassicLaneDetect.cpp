/**
 * @brief 传统车道线检测功能
 * @file funcClassicLaneDetect.cpp
 * @version 0.0.1
 * @author Chi Ding (dc22@mails.tsinghua.edu.com)
 * @date 2023-12-28
 */
#include "funcClassicLaneDetect.hpp"


/**
* @brief classicLaneDetectRun函数
* @param[IN] param1,空参数
* @param[IN] input1, classicLaneDetect类的对象
* @param[OUT] output1, 空输出

* @cn_name" 运行车道线检测

* @granularity: atomic 

* @tag: perception
*/
void classicLaneDetectRun(ParamStruct01 &param01, InputStruct01 &input01, OutputStruct01 &output01)
{
    ClassicLaneDetect &laneDetect = input01.laneDetect;
    laneDetect.run();
}

/**
* @brief classicLaneDetectPolyFit函数
* @param[IN] param2, 自变量数据，多项式次数，因变量数据
* @param[IN] input2, classicLaneDetect类的对象
* @param[OUT] output2, 空输出

* @cn_name: 车道线关键点拟合

* @granularity: atomic 

* @tag: perception
*/
void classicLaneDetectPolyFit(ParamStruct02 &param02, InputStruct02 &input02, OutputStruct02 &output02)
{
    ClassicLaneDetect &laneDetect = input02.laneDetect;
    vector<Point> &chain = param02.chain;
    int n = param02.n;
    Mat &a = param02.a;
    laneDetect.polyFit(chain,n,a);
}

/**
* @brief classicLaneDetectGetPerspectiveMatrix函数
* @param[IN] param3,原始图片
* @param[IN] input3, classicLaneDetect类的对象
* @param[OUT] output3, 空输出

* @cn_name: 映射透视矩阵

* @granularity: atomic 

* @tag: perception
*/
void classicLaneDetectGetPerspectiveMatrix(ParamStruct03 &param03, InputStruct03 &input03, OutputStruct03 &output03)
{
    ClassicLaneDetect &laneDetect = input03.laneDetect;
    Mat &originImage = param03.originImage;
    laneDetect.getPerspectiveMatrix(originImage);
}

/**
* @brief classicLaneDetectLightCompensate函数
* @param[IN] param4,原始图片
* @param[IN] input4, classicLaneDetect类的对象
* @param[OUT] output4, 补偿后图片

* @cn_name: 光补偿

* @granularity: atomic 

* @tag: perception
*/
void classicLaneDetectLightCompensate (ParamStruct04 &param04, InputStruct04 &input04, OutputStruct04 &output04)
{
    ClassicLaneDetect &laneDetect = input04.laneDetect;
    Mat &originImage = param04.originImage;
    Mat compensateImage;
    laneDetect.lightCompensate(originImage,compensateImage);
    output04.compensateImage = compensateImage;
}

/**
* @brief classicLaneDetectImageEnhancement函数
* @param[IN] param5,原始图片
* @param[IN] input5, classicLaneDetect类的对象
* @param[OUT] output5, 增强后图片

* @cn_name: 图片增强

* @granularity: atomic 

* @tag: perception
*/
void classicLaneDetectImageEnhancement (ParamStruct05 &param05, InputStruct05 &input05, OutputStruct05 &output05)
{
    ClassicLaneDetect &laneDetect = input05.laneDetect;
    Mat &originImage = param05.originImage;
    Mat enhancedImage;
    laneDetect.imageEnhancement(originImage,enhancedImage);
    output05.enhancedImage = enhancedImage;
}

/**
* @brief classicLaneDetectPreProcess函数
* @param[IN] param6,原始图片
* @param[IN] input6, classicLaneDetect类的对象
* @param[OUT] output6, 预处理后图片

* @cn_name: 图像预处理

* @granularity: atomic 

* @tag: perception
*/
void classicLaneDetectPreProcess (ParamStruct06 &param06, InputStruct06 &input06, OutputStruct06 &output06)
{
    ClassicLaneDetect &laneDetect = input06.laneDetect;
    Mat &originImage = param06.originImage;
    Mat processImage;
    laneDetect.preProcess(originImage,processImage);
    output06.processImage = processImage;
}

/**
* @brief classicLaneDetectDetectLaneByEdges函数
* @param[IN] param7,原始图片,左车道线左边界点，左车道线右边界点，右车道线左边界点，右车道线右边界点
* @param[IN] input7, classicLaneDetect类的对象
* @param[OUT] output7, 预处理后图片

* @cn_name: 依据边界的车道线检测

* @granularity: atomic 

* @tag: perception
*/
void classicLaneDetectDetectLaneByEdges (ParamStruct07 &param07, InputStruct07 &input07, OutputStruct07 &output07)
{
    ClassicLaneDetect &laneDetect = input07.laneDetect;
    Mat &image = param07.image;
    vector<Point> &leftPoints1 = param07.leftPoints1;
    vector<Point> &leftPoints2 = param07.leftPoints2;
    vector<Point> &rightPoints1 = param07.rightPoints1;
    vector<Point> &rightPoints2 = param07.rightPoints2;
    laneDetect.detectLaneByEdges(image,leftPoints1,leftPoints2,rightPoints1,rightPoints2);
}

/**
* @brief classicLaneDetectGenerateFinalLanes函数
* @param[IN] param8, 左车道线左边界点，左车道线右边界点，右车道线左边界点，右车道线右边界点
* @param[IN] input8, classicLaneDetect类的对象
* @param[OUT] output8, 左车道线参数，右车道线参数

* @cn_name: 最终左右车道线参数

* @granularity: atomic 

* @tag: perception
*/
void classicLaneDetectGenerateFinalLanes (ParamStruct08 &param08, InputStruct08 &input08, OutputStruct08 &output08)
{
    ClassicLaneDetect &laneDetect = input08.laneDetect;
    vector<Point> &leftPoints1 = param08.leftPoints1;
    vector<Point> &leftPoints2 = param08.leftPoints2;
    vector<Point> &rightPoints1 = param08.rightPoints1;
    vector<Point> &rightPoints2 = param08.rightPoints2;
    Mat leftFunc;
    Mat rightFunc;
    laneDetect.generateFinalLanes(leftPoints1,leftPoints2,rightPoints1,rightPoints2,leftFunc,rightFunc);
    output08.leftFunc = leftFunc;
    output08.rightFunc = rightFunc;
}

/**
* @brief classicLaneDetectDrawLane函数
* @param[IN] param9, 原始图片，车道线绘制参数
* @param[IN] input9, classicLaneDetect类的对象
* @param[OUT] output9, 空输出

* @cn_name: 绘制车道线

* @granularity: atomic 

* @tag: perception
*/
void classicLaneDetectDrawLane (ParamStruct09 &param09, InputStruct09 &input09, OutputStruct09 &output09)
{
    ClassicLaneDetect &laneDetect = input09.laneDetect;
    Mat showImage = param09.showImage;
    Mat laneFunc = param09.laneFunc;
    laneDetect.drawLane(showImage,laneFunc);
}

/**
* @brief classicLaneDetectDisplayLanes函数
* @param[IN] param10, 空参数
* @param[IN] input10, classicLaneDetect类的对象
* @param[OUT] output10, 空输出

* @cn_name: 呈现车道线

* @granularity: atomic 

* @tag: perception
*/
void classicLaneDetectDisplayLanes (ParamStruct10 &param10, InputStruct10 &input10, OutputStruct10 &output10)
{
    ClassicLaneDetect &laneDetect = input10.laneDetect;
    laneDetect.displayLanes();
}

/**
* @brief classicLaneDetectModuleSelfCheck函数
* @param[IN] param11, 空参数
* @param[IN] input11, classicLaneDetect类的对象
* @param[OUT] output11, 空输出

* @cn_name: 车道线参数确认

* @granularity: atomic 

* @tag: perception
*/
void classicLaneDetectModuleSelfCheck (ParamStruct11 &param11, InputStruct11 &input11, OutputStruct11 &output11)
{
    ClassicLaneDetect &laneDetect = input11.laneDetect;
    laneDetect.moduleSelfCheck();
}

/**
* @brief classicLaneDetectModuleSelfCheckPrint函数
* @param[IN] param12, 空参数
* @param[IN] input12, classicLaneDetect类的对象
* @param[OUT] output12, 空输出

* @cn_name: 车道线参数确认打印

* @granularity: atomic 

* @tag: perception
*/
void classicLaneDetectModuleSelfCheck (ParamStruct12 &param12, InputStruct12 &input12, OutputStruct12 &output12)
{
    ClassicLaneDetect &laneDetect = input12.laneDetect;
    laneDetect.moduleSelfCheckPrint();
}

/**
* @brief classicLaneDetectGetImagesName函数
* @param[IN] param13, 图片路径，图片名称
* @param[IN] input13, 空输入
* @param[OUT] output13, 空输出

* @cn_name: 获取图片名称

* @granularity: atomic 

* @tag: perception
*/
void classicLaneDetectGetImagesName (ParamStruct13 &param13, InputStruct13 &input13, OutputStruct13 &output13)
{
    string path = param13.path;
    vector<string> &imageNames = param13.imageNames;
    getImagesName(path,imageNames);
}