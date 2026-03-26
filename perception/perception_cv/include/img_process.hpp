#ifndef IMG_PROCESS_H
#define IMG_PROCESS_H

#include <opencv2/opencv.hpp>

/**
 * \namespace perception
 * \brief perception namespace
 */
namespace perception{
bool loadImage(const char *filename, cv::Mat &clr_image);
bool saveImage(cv::Mat input, const char *filename);
void imShow(cv::Mat input, const char *window_name = "img");

bool imGray(cv::Mat input, cv::Mat &output);

void imScharrTrans(cv::Mat input, cv::Mat &output, double scale = 1, double delta = 0);

bool imEqualizeHistogram(cv::Mat input, cv::Mat &output);

bool imGammaCorrection(cv::Mat input, cv::Mat &output, double gamma);

bool imInvertTransform(cv::Mat input, cv::Mat &output);

bool imLogTransform(cv::Mat input, cv::Mat &output, double c);  // to be tested

//Filtering
bool imMeanFilter(cv::Mat input, cv::Mat &output, int kernel_size_x = 3, int kernel_size_y = 3);
bool imMedianFilter(cv::Mat input, cv::Mat &output, int kernel_size = 3);
void imGaussianFilter(cv::Mat input, cv::Mat &output, int kernel_size = 3, double sigma = 1.0);
void imSharpeningFilter(cv::Mat input, cv::Mat &output, double sigma = 1.0, double amount = 1.5, int threshold = 0);
bool imBilateralFilter(cv::Mat input, cv::Mat &output, int diameter = 5, double sigma_color = 75.0, double sigma_space = 75.0);

void imDrawRectangle (cv::Mat input, cv::Rect rect, cv::Scalar color=CV_RGB(255,0,0), int width=1);
void imDrawText(cv::Mat input, char* str, cv::Point point, int fontFace = cv::FONT_HERSHEY_SIMPLEX, 
                 float size=.25f, cv::Scalar color=CV_RGB(1,1,1));
void imDrawCircle(cv::Mat input, cv::Point center, int radius, 
                  cv::Scalar color=CV_RGB(255,0,0), int thickness=1,
                  int lineType=cv::LINE_8, int shift=0);

void imLaplacian(cv::Mat input, cv::Mat &output, int kernel_size = 3,double scale = 1, double delta = 0);

void imDoubleThresholding(cv::Mat input, cv::Mat &output, 
                         double low_threshold = 20, double high_threshold = 180,
                         uchar strong_pixel = 255, uchar weak_pixel = 128);

void imRoberts(cv::Mat input, cv::Mat &output);

void imPrewitt(cv::Mat input, cv::Mat &output);

//Morphology
void imMorphologyDilation(cv::Mat input, cv::Mat &output, int kernel_size = 3);

void imMorphologyErosion(cv::Mat input, cv::Mat &output, int kernel_size = 3);

void imMorphologyOpen(cv::Mat input, cv::Mat &output, int kernel_size = 3);

void imMorphologyClose(cv::Mat input, cv::Mat &output, int kernel_size = 3);

//insert
void imNearestInterpolation(cv::Mat input, cv::Mat &output, double sx, double sy);

void imBiLineInterpolate(cv::Mat input, cv::Mat &output, double scale_x,double scale_y);

void imCubicInterpolation(cv::Mat input, cv::Mat &output, double scale_x,double scale_y);

void imGetNearestNeighbor(cv::Mat input, uchar& result, double pos_x,double pos_y);

void imGetBilinearInterpolation(cv::Mat input, uchar& result, double pos_x,double pos_y);

//Contour Detection
void imGetContours(cv::Mat input, cv::Mat &output, std::vector<std::vector<cv::Point>> &contours,
                  int mode = cv::RETR_EXTERNAL,
                  int method = cv::CHAIN_APPROX_SIMPLE);
// write a function whoes name is pcdBIRCHCluster to perform   algorithm with opencv, and it's input is cv::Mat input, output is cv::Mat &output, and you can add other input you need, and return void. And add note with doxygen format
}
#endif
