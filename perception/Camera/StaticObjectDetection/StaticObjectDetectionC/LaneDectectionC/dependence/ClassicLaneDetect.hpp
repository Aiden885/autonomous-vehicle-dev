/***
 * @Author: Sheree
 * @Date: 2022-08-05 02:00:26
 * @LastEditors: Sheree
 * @LastEditTime: 2022-09-13 19:17:37
 * @FilePath: \LaneDetect_v_cpp\include\ClassicLaneDetect.hpp
 * @Description:
 * @
 * @Copyright (c) 2022 by Sheree, All Rights Reserved.
 */
#include <opencv2/opencv.hpp>
#include <opencv2/highgui.hpp>
#include <vector>
using namespace cv;
using namespace std;
struct parameters
{
    int height = 300;
    int width = 500;
    int blockSize = 10;
    int leftClassifyMargin = 10;
    int rightClassifyMargin = 12;
    vector<Point> initROIZone = {Point(0, height), Point(0, 170), Point(280, 20), Point(330, 0), Point(360, 0), Point(640, 280), Point(640, height)};
    vector<Point> initROIMask = {Point(50, height), Point(350, 60), Point(500, height)};
    vector<vector<Point>> initROI = {initROIZone, initROIMask};
    double gamma = 3;
};

struct mouseParam
{
    Mat src;
    Mat show;
    Mat matrix;
    vector<Point> pts;
    int var = 0;
    int flag = 0;
};

class ClassicLaneDetect
{
public:
    Mat image;
    int imageSize[2] = {480, 640};
    Mat leftFunc, rightFunc;

private:
    Mat warpPerspectiveMatrix = Mat(3, 3, CV_32F);
    parameters processParameters;
    int fitOrder = 1;

public:
    ~ClassicLaneDetect();
    void run();
    void moduleSelfCheck();
    void moduleSelfCheckPrint();
    void getPerspectiveMatrix(Mat &originImage);
    void lightCompensate(Mat &originImage, Mat &compensateImage);
    void imageEnhancement(Mat &originImage, Mat &enhancedImage);
    void preProcess(Mat &originImage, Mat &processImage);
    void drawLane(Mat &originImage, Mat &laneFunc);
    void displayLanes();
    void detectLaneByEdges(Mat &image, vector<Point> &leftPoints1, vector<Point> &leftPoints2, vector<Point> &rightPoints1, vector<Point> &rightPoints2);
    // void detectLaneByWindows(Mat &image, vector<Point> &leftLane, vector<Point> &rightLane);
    void generateFinalLanes(vector<Point> &leftPoints1, vector<Point> &leftPoints2, vector<Point> &rightPoints1, vector<Point> &rightPoints2, Mat &leftFunc, Mat &rightFunc);
    void polyFit(vector<Point> &chain, int n, Mat &a);
};
