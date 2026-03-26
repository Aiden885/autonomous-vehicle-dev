/**
 * @brief 基于深度学习方法的车道线检测功能
 * @file funcUFLD.cpp
 * @version 0.0.1
 * @author Chi Ding (dc22@mails.tsinghua.edu.com)
 * @date 2024-01-05
 */
 
 #include "funcUFLD.hpp"


 /**
* @brief funcUFLDLaneDetectReadConfig函数
* @param[IN] param1, 配置文件文件名
* @param[IN] input1, UFLDLaneDetector类的对象
* @param[OUT] output1, 空输出

* @cn_name: 读取配置

* @granularity: atomic 

* @tag: perception
*/
void funcUFLDLaneDetectReadConfig (ParamStruct01 &param01, InputStruct01 &input01, OutputStruct01 &output01)
{
    UFLDLaneDetector &UFLDlaneDetect = input01.UFLDlaneDetect;
    string configFile = param01.configFile;
    UFLDlaneDetect.readConfig(configFile);
}

/**
* @brief funcUFLDLaneDetectInitTRTmodel函数
* @param[IN] param2, 模型文件文件名
* @param[IN] input2, UFLDLaneDetector类的对象
* @param[OUT] output2, 空输出

* @cn_name: 初始化模型

* @granularity: atomic 

* @tag: perception
*/
void funcUFLDLaneDetectInitTRTmodel (ParamStruct02 &param02, InputStruct02 &input02, OutputStruct02 &output02)
{
    UFLDLaneDetector &UFLDlaneDetect = input02.UFLDlaneDetect;
    string engineFile = param02.engineFile;
    UFLDlaneDetect.initTRTmodel(engineFile);
}

/**
* @brief funcUFLDLaneDetectInitCalib函数
* @param[IN] param3, 校准文件文件名
* @param[IN] input3, UFLDLaneDetector类的对象
* @param[OUT] output3, 空输出

* @cn_name: 初始化校准

* @granularity: atomic 

* @tag: perception
*/
void funcUFLDLaneDetectInitCalib (ParamStruct03 &param03, InputStruct03 &input03, OutputStruct03 &output03)
{
    UFLDLaneDetector &UFLDlaneDetect = input03.UFLDlaneDetect;
    string calibFile = param03.calibFile;
    UFLDlaneDetect.initCalib(calibFile);
}

/**
* @brief funcUFLDLaneDetectPreProcess函数
* @param[IN] param4, 图像，预处理参数
* @param[IN] input4, UFLDLaneDetector类的对象
* @param[OUT] output4, 空输出

* @cn_name: 图像预处理

* @granularity: atomic 

* @tag: perception
*/
void funcUFLDLaneDetectPreProcess (ParamStruct04 &param04, InputStruct04 &input04, OutputStruct04 &output04)
{
    UFLDLaneDetector &UFLDlaneDetect = input04.UFLDlaneDetect;
    Mat &img = param04.img;
    vector<float> &processedImg = param04.processedImg;
    UFLDlaneDetect.preProcess(img,processedImg);
}

/**
* @brief funcUFLDLaneDetectDoInference函数
* @param[IN] param5, 推理执行，输入尺寸，输出尺寸，数据批量大小
* @param[IN] input5, UFLDLaneDetector类的对象
* @param[OUT] output5, 空输出

* @cn_name: 模型推理

* @granularity: atomic 

* @tag: perception
*/
void funcUFLDLaneDetectDoInference (ParamStruct05 &param05, InputStruct05 &input05, OutputStruct05 &output05)
{
    UFLDLaneDetector &UFLDlaneDetect = input05.UFLDlaneDetect;
    IExecutionContext &context = param05.context;
    float input = param05.input;
    float output = param05.output;
    int batchSize = param05.batchSize;
    UFLDlaneDetect.doInference(context,input,output,batchSize);
}

/**
* @brief funcUFLDLaneDetectSoftmax_mul函数
* @param[IN] param6, 损失函数参数
* @param[IN] input6, UFLDLaneDetector类的对象
* @param[OUT] output6, 空输出

* @cn_name: 损失函数计算

* @granularity: atomic 

* @tag: perception
*/
void funcUFLDLaneDetectSoftmax_mul (ParamStruct06 &param06, InputStruct06 &input06, OutputStruct06 &output06)
{
    UFLDLaneDetector &UFLDlaneDetect = input06.UFLDlaneDetect;
    float x = param06.x;
    float y = param06.y;
    int rows = param06.rows;
    int cols = param06.cols;
    int chan = param06.chan;
    UFLDlaneDetect.softmax_mul(x,y,rows,cols,chan);
}

/**
* @brief funcUFLDLaneDetectArgmax函数
* @param[IN] param7, 优化函数参数
* @param[IN] input7, UFLDLaneDetector类的对象
* @param[OUT] output7, 空输出

* @cn_name: 最大值索引优化

* @granularity: atomic 

* @tag: perception
*/
void funcUFLDLaneDetectArgmax (ParamStruct07 &param07, InputStruct07 &input07, OutputStruct07 &output07)
{
    UFLDLaneDetector &UFLDlaneDetect = input07.UFLDlaneDetect;
    float x = param07.x;
    float y = param07.y;
    int rows = param07.rows;
    int cols = param07.cols;
    int chan = param07.chan;
    UFLDlaneDetect.argmax(x,y,rows,cols,chan);
}

/**
* @brief funcUFLDLaneDetectPostProcess函数
* @param[IN] param8, 空参数
* @param[IN] input8, UFLDLaneDetector类的对象
* @param[OUT] output8, 空输出

* @cn_name: 后处理

* @granularity: atomic 

* @tag: perception
*/
void funcUFLDLaneDetectPostProcess (ParamStruct08 &param08, InputStruct08 &input08, OutputStruct08 &output08)
{
    UFLDLaneDetector &UFLDlaneDetect = input08.UFLDlaneDetect;
    UFLDlaneDetect.postProcess();
}

/**
* @brief funcUFLDLaneDetectDisplay函数
* @param[IN] param9, 空参数
* @param[IN] input9, UFLDLaneDetector类的对象
* @param[OUT] output9, 是否打印

* @cn_name: 车道线打印

* @granularity: atomic 

* @tag: perception
*/
void funcUFLDLaneDetectDisplay (ParamStruct09 &param09, InputStruct09 &input09, OutputStruct09 &output09)
{
    UFLDLaneDetector &UFLDlaneDetect = input09.UFLDlaneDetect;
    bool outputlane = UFLDlaneDetect.display();
    output.outputlane = outputlane;
}

/**
* @brief funcUFLDLaneDetectToProto函数
* @param[IN] param10, 空参数
* @param[IN] input10, UFLDLaneDetector类的对象
* @param[OUT] output10, 空输出

* @cn_name: 权重参数

* @granularity: atomic 

* @tag: perception
*/
void funcUFLDLaneDetectToProto (ParamStruct10 &param10, InputStruct10 &input10, OutputStruct10 &output10)
{
    UFLDLaneDetector &UFLDlaneDetect = input10.UFLDlaneDetect;
    UFLDlaneDetect.toProto();
}

/**
* @brief funcUFLDLaneDetectRun函数
* @param[IN] param11, 空参数
* @param[IN] input11, UFLDLaneDetector类的对象
* @param[OUT] output11, 空输出

* @cn_name: 运行函数

* @granularity: atomic 

* @tag: perception
*/
void funcUFLDLaneDetectRun (ParamStruct11 &param11, InputStruct11 &input11, OutputStruct11 &output11)
{
    UFLDLaneDetector &UFLDlaneDetect = input11.UFLDlaneDetect;
    UFLDlaneDetect.run();
}