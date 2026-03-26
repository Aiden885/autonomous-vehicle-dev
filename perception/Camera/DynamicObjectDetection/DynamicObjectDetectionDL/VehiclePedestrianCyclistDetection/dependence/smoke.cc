#include "smoke.hh"

#include <fstream>
#include <memory>
#include <NvInferPlugin.h>
#include <cuda_runtime_api.h>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

#define IMAGE_H 375
#define IMAGE_W 1242
#define INPUT_H 384
#define INPUT_W 1280
#define OUTPUT_H (INPUT_H / 4)
#define OUTPUT_W (INPUT_W / 4)
#define SCORE_THRESH 0.3f
#define TOPK 100

SMOKE::SMOKE(const std::string &enginePath, const cv::Mat &intrinsic)
{
    bufferSize[0] = 3 * INPUT_H * INPUT_W;
    bufferSize[1] = TOPK * 8;
    bufferSize[2] = TOPK;
    bufferSize[3] = TOPK;
    cudaMalloc(&buffers_[0], bufferSize[0] * sizeof(float));
    cudaMalloc(&buffers_[1], bufferSize[1] * sizeof(float));
    cudaMalloc(&buffers_[2], bufferSize[2] * sizeof(float));
    cudaMalloc(&buffers_[3], bufferSize[3] * sizeof(float));
    imageData.resize(bufferSize[0]);
    bboxPreds.resize(bufferSize[1]);
    topkScores.resize(bufferSize[2]);
    topkIndices.resize(bufferSize[3]);
    cudaStreamCreate(&stream_);

    // https://github.com/open-mmlab/mmdetection3d/blob/master/configs/_base_/models/smoke.py#L41
    baseDepth = {28.01f, 16.32f};
    baseDims.resize(3); // pedestrian, cyclist, car
    baseDims[0].x = 0.88f;
    baseDims[0].y = 1.73f;
    baseDims[0].z = 0.67f;
    baseDims[1].x = 1.78f;
    baseDims[1].y = 1.70f;
    baseDims[1].z = 0.58f;
    baseDims[2].x = 3.88f;
    baseDims[2].y = 1.63f;
    baseDims[2].z = 1.53f;

    // loadEngine(enginePath);
}

SMOKE::~SMOKE()
{
    cudaStreamDestroy(stream_);
    for (auto &buffer : buffers_)
    {
        cudaFree(buffer);
    }
    if (context_ != nullptr)
    {
        context_->destroy();
        engine_->destroy();
    }
}

void SMOKE::intrinsicInit(const cv::Mat &intrinsic)
{
    // Modify camera intrinsics due to scaling
    intrinsic_ = intrinsic.clone();
    intrinsic_.at<float>(0, 0) *= static_cast<float>(INPUT_W) / IMAGE_W;
    intrinsic_.at<float>(0, 2) *= static_cast<float>(INPUT_W) / IMAGE_W;
    intrinsic_.at<float>(1, 1) *= static_cast<float>(INPUT_H) / IMAGE_H;
    intrinsic_.at<float>(1, 2) *= static_cast<float>(INPUT_H) / IMAGE_H;
}
void SMOKE::preProcess(cv::Mat &imgResize)
{ // Preprocessing
    // resize尺度变换
    // cv::resize(input, imgResize, cv::Size(INPUT_W, INPUT_H), cv::INTER_LINEAR);

    // 色彩空间变换
    float mean[3]{123.675f, 116.280f, 103.530f};
    float std[3] = {58.395f, 57.120f, 57.375f};
    uint8_t *dataHWC = reinterpret_cast<uint8_t *>(imgResize.data);
    float *dataCHW = imageData.data();
    for (int c = 0; c < 3; ++c)
    {
        for (unsigned j = 0, imgSize = INPUT_H * INPUT_W; j < imgSize; ++j)
        {
            dataCHW[c * imgSize + j] = (dataHWC[j * 3 + 2 - c] - mean[c]) / std[c]; // bgr2rgb
        }
    }
}
void SMOKE::inference()
{
    // Preprocessing
    // cv::Mat imgResize;
    // preProcess(cv::Mat & imgResize);

    // Do inference
    cudaMemcpyAsync(buffers_[0], imageData.data(), bufferSize[0] * sizeof(float), cudaMemcpyHostToDevice, stream_);
    context_->executeV2(&buffers_[0]);
    cudaMemcpyAsync(bboxPreds.data(), buffers_[1], bufferSize[1] * sizeof(float), cudaMemcpyDeviceToHost, stream_);
    cudaMemcpyAsync(topkScores.data(), buffers_[2], bufferSize[2] * sizeof(float), cudaMemcpyDeviceToHost, stream_);
    cudaMemcpyAsync(topkIndices.data(), buffers_[3], bufferSize[3] * sizeof(float), cudaMemcpyDeviceToHost, stream_);
    // cudaStreamSynchronize(stream_);

    // Decoding and visualization
    // postProcess(imgResize);
}

void SMOKE::loadEngine(const std::string &enginePath)
{
    std::ifstream inFile(enginePath, std::ios::binary);
    if (!inFile.is_open())
    {
        std::cerr << "Failed to open engine file: " << enginePath << std::endl;
        return;
    }
    inFile.seekg(0, inFile.end);
    int length = inFile.tellg();
    inFile.seekg(0, inFile.beg);
    std::unique_ptr<char[]> trtModelStream(new char[length]);
    inFile.read(trtModelStream.get(), length);
    inFile.close();

    // getPluginCreator could not find plugin: MMCVModulatedDeformConv2d version: 1
    initLibNvInferPlugins(&gLogger, "");
    nvinfer1::IRuntime *runtime = nvinfer1::createInferRuntime(gLogger);
    assert(runtime != nullptr);
    engine_ = runtime->deserializeCudaEngine(trtModelStream.get(), length, nullptr);
    assert(engine_ != nullptr);
    context_ = engine_->createExecutionContext();
    assert(context_ != nullptr);

    runtime->destroy();
}

float Sigmoid(float x)
{
    return 1.0f / (1.0f + expf(-x));
}

void SMOKE::bBoxPositionTrans(int i, float &x, float &y, float &z)
{
    int class_id = static_cast<int>(topkIndices[i] / OUTPUT_H / OUTPUT_W);
    int location = static_cast<int>(topkIndices[i]) % (OUTPUT_H * OUTPUT_W);
    int img_x = location % OUTPUT_W;
    int img_y = location / OUTPUT_W;
    // Depth
    z = baseDepth[0] + bboxPreds[8 * i] * baseDepth[1];
    // Location
    cv::Mat imgPoint(3, 1, CV_32FC1);
    imgPoint.at<float>(0) = 4.0f * (static_cast<float>(img_x) + bboxPreds[8 * i + 1]);
    imgPoint.at<float>(1) = 4.0f * (static_cast<float>(img_y) + bboxPreds[8 * i + 2]);
    imgPoint.at<float>(2) = 1.0f;
    cv::Mat camPoint = intrinsic_.inv() * imgPoint * z;
    x = camPoint.at<float>(0);
    y = camPoint.at<float>(1);
}

void SMOKE::bBoxSizeTrans(int i, float x, float y, float z, float &w, float &l, float &h)
{
    // Dimension
    float w = baseDims[class_id].x * expf(Sigmoid(bboxPreds[8 * i + 3]) - 0.5f);
    float l = baseDims[class_id].y * expf(Sigmoid(bboxPreds[8 * i + 4]) - 0.5f);
    float h = baseDims[class_id].z * expf(Sigmoid(bboxPreds[8 * i + 5]) - 0.5f);
}
void SMOKE::bBoxAngle(int i, float x, float y, float z, float &angle)
{
    // Orientation
    float oriNorm = sqrtf(powf(bboxPreds[8 * i + 6], 2.0f) + powf(bboxPreds[8 * i + 7], 2.0f));
    bboxPreds[8 * i + 6] /= oriNorm; // sin(alpha)
    bboxPreds[8 * i + 7] /= oriNorm; // cos(alpha)
    float ray = atan(x / (z + 1e-7f));
    float alpha = atan(bboxPreds[8 * i + 6] / (bboxPreds[8 * i + 7] + 1e-7f));
    if (bboxPreds[8 * i + 7] > 0.0f)
    {
        alpha -= mPi / 2.0f;
    }
    else
    {
        alpha += mPi / 2.0f;
    }
    angle = alpha + ray;
    if (angle > mPi)
    {
        angle -= 2.0f * mPi;
    }
    else if (angle < -mPi)
    {
        angle += 2.0f * mPi;
    }

    // https://github.com/open-mmlab/mmdetection3d/blob/master/mmdet3d/core/bbox/structures/cam_box3d.py#L117
    //              front z
    //                   /
    //                  /
    //    (x0, y0, z1) + -----------  + (x1, y0, z1)
    //                /|            / |
    //               / |           /  |
    // (x0, y0, z0) + ----------- +   + (x1, y1, z1)
    //              |  /      .   |  /
    //              | / origin    | /
    // (x0, y1, z0) + ----------- + -------> x right
    //              |             (x1, y1, z0)
    //              |
    //              v
    //         down y
}
void SMOKE::bBoxCam2Image(cv::Mat &inputImg)
{
    cv::Mat camCorners = (cv::Mat_<float>(8, 3) << -w, -l, -h, // (x0, y0, z0)
                          -w, -l, h,                           // (x0, y0, z1)
                          -w, l, h,                            // (x0, y1, z1)
                          -w, l, -h,                           // (x0, y1, z0)
                          w, -l, -h,                           // (x1, y0, z0)
                          w, -l, h,                            // (x1, y0, z1)
                          w, l, h,                             // (x1, y1, z1)
                          w, l, -h);                           // (x1, y1, z0)
    camCorners = 0.5f * camCorners;
    cv::Mat rotationY = cv::Mat::eye(3, 3, CV_32FC1);
    rotationY.at<float>(0, 0) = cosf(angle);
    rotationY.at<float>(0, 2) = sinf(angle);
    rotationY.at<float>(2, 0) = -sinf(angle);
    rotationY.at<float>(2, 2) = cosf(angle);
    // cos, 0, sin
    //   0, 1,   0
    //-sin, 0, cos
    camCorners = camCorners * rotationY.t();
    for (int i = 0; i < 8; ++i)
    {
        camCorners.at<float>(i, 0) += x;
        camCorners.at<float>(i, 1) += y;
        camCorners.at<float>(i, 2) += z;
    }
    camCorners = camCorners * intrinsic_.t();
    std::vector<cv::Point2f> imgCorners(8);
    for (int i = 0; i < 8; ++i)
    {
        imgCorners[i].x = camCorners.at<float>(i, 0) / camCorners.at<float>(i, 2);
        imgCorners[i].y = camCorners.at<float>(i, 1) / camCorners.at<float>(i, 2);
    }
    for (int i = 0; i < 4; ++i)
    {
        const auto &p1 = imgCorners[i];
        const auto &p2 = imgCorners[(i + 1) % 4];
        const auto &p3 = imgCorners[i + 4];
        const auto &p4 = imgCorners[(i + 1) % 4 + 4];
        cv::line(inputImg, p1, p2, cv::Scalar(241, 101, 72), 1, cv::LINE_AA);
        cv::line(inputImg, p3, p4, cv::Scalar(241, 101, 72), 1, cv::LINE_AA);
        cv::line(inputImg, p1, p3, cv::Scalar(241, 101, 72), 1, cv::LINE_AA);
    }

    cv::imshow("SMOKE_TRT", inputImg);
    cv::waitKey(0);
}
// void SMOKE::postProcess(cv::Mat &inputImg)
// {
//     for (int i = 0; i < TOPK; ++i)
//     {
//         if (topkScores[i] < SCORE_THRESH)
//         {
//             continue;
//         }

//         float x, y, z = 0.0;
//         bBoxPositionTrans(i, x, y, z);
//         float w, l, h = 0.0;
//         bBoxSizeTrans(i, x, y, z, w, l, h);
//         float angle = 0.0;
//         bBoxAngle(i, x, y, z, angle);

//         cv::Mat camCorners = (cv::Mat_<float>(8, 3) << -w, -l, -h, // (x0, y0, z0)
//                               -w, -l, h,                           // (x0, y0, z1)
//                               -w, l, h,                            // (x0, y1, z1)
//                               -w, l, -h,                           // (x0, y1, z0)
//                               w, -l, -h,                           // (x1, y0, z0)
//                               w, -l, h,                            // (x1, y0, z1)
//                               w, l, h,                             // (x1, y1, z1)
//                               w, l, -h);                           // (x1, y1, z0)
//         camCorners = 0.5f * camCorners;
//         cv::Mat rotationY = cv::Mat::eye(3, 3, CV_32FC1);
//         rotationY.at<float>(0, 0) = cosf(angle);
//         rotationY.at<float>(0, 2) = sinf(angle);
//         rotationY.at<float>(2, 0) = -sinf(angle);
//         rotationY.at<float>(2, 2) = cosf(angle);
//         // cos, 0, sin
//         //   0, 1,   0
//         //-sin, 0, cos
//         camCorners = camCorners * rotationY.t();
//         for (int i = 0; i < 8; ++i)
//         {
//             camCorners.at<float>(i, 0) += x;
//             camCorners.at<float>(i, 1) += y;
//             camCorners.at<float>(i, 2) += z;
//         }
//         camCorners = camCorners * intrinsic_.t();
//         std::vector<cv::Point2f> imgCorners(8);
//         for (int i = 0; i < 8; ++i)
//         {
//             imgCorners[i].x = camCorners.at<float>(i, 0) / camCorners.at<float>(i, 2);
//             imgCorners[i].y = camCorners.at<float>(i, 1) / camCorners.at<float>(i, 2);
//         }
//         for (int i = 0; i < 4; ++i)
//         {
//             const auto &p1 = imgCorners[i];
//             const auto &p2 = imgCorners[(i + 1) % 4];
//             const auto &p3 = imgCorners[i + 4];
//             const auto &p4 = imgCorners[(i + 1) % 4 + 4];
//             cv::line(inputImg, p1, p2, cv::Scalar(241, 101, 72), 1, cv::LINE_AA);
//             cv::line(inputImg, p3, p4, cv::Scalar(241, 101, 72), 1, cv::LINE_AA);
//             cv::line(inputImg, p1, p3, cv::Scalar(241, 101, 72), 1, cv::LINE_AA);
//         }
//     }
//     cv::imshow("SMOKE_TRT", inputImg);
//     cv::waitKey(0);
// }
