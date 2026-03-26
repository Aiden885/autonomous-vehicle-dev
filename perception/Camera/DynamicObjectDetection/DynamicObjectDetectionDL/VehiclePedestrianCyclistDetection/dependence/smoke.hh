#pragma once

#include <iostream>
#include <NvInfer.h>
#include <opencv2/core.hpp>
#include "../../include/baseCameraObjectDetector.h"

class Logger : public nvinfer1::ILogger
{
public:
    explicit Logger(Severity severity = Severity::kWARNING)
        : reportableSeverity(severity) {}

    void log(Severity severity, const char *msg) noexcept
    {
        if (severity > reportableSeverity)
        {
            return;
        }
        switch (severity)
        {
        case Severity::kINTERNAL_ERROR:
            std::cerr << "INTERNAL_ERROR: ";
            break;
        case Severity::kERROR:
            std::cerr << "ERROR: ";
            break;
        case Severity::kWARNING:
            std::cerr << "WARNING: ";
            break;
        case Severity::kINFO:
            std::cerr << "INFO: ";
            break;
        default:
            std::cerr << "UNKNOWN: ";
            break;
        }
        std::cerr << msg << std::endl;
    }
    Severity reportableSeverity;
};

struct BboxDim
{
    float x;
    float y;
    float z;
};

class SMOKE
{
public:
    SMOKE();

    ~SMOKE();

    void intrinsicInit(const cv::Mat &intrinsic);

    void inference();

    void loadEngine(const std::string &enginePath);

    void postProcess(cv::Mat &inputImg);
    void preProcess(cv::Mat &input, cv::Mat &inputImg);
    void bBoxPositionTrans(int i, float &x, float &y, float &z);
    void bBoxSizeTrans(int i, float x, float y, float z, float &w, float &l, float &h);
    void bBoxAngle(int i, float x, float y, float z, float &angle);

private:
    Logger gLogger_;
    cudaStream_t stream_;
    nvinfer1::ICudaEngine *engine_;
    nvinfer1::IExecutionContext *context_;

    void *buffers_[4];
    int bufferSize[4];
    std::vector<float> imageData;
    std::vector<float> bboxPreds;
    std::vector<float> topkScores;
    std::vector<float> topkIndices;
    cv::Mat intrinsic_;
    std::vector<float> baseDepth;
    std::vector<BboxDim> baseDims;
};
