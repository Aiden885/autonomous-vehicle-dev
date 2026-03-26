#include "../include/ClassicLaneDetect.hpp"
using namespace std;
mouseParam param;
/***
 * @description: polyfit using fit points
 * @param [vector<Point2f>] &chain: fit points
 * @param [int] n: order
 * @param [Mat] &a: function parameters
 * @return []
 */
void ClassicLaneDetect::polyFit(vector<Point> &chain, int n, Mat &a)
{
    Mat y(chain.size(), 1, CV_32F, Scalar::all(0));
    Mat phy(chain.size(), n, CV_32F, Scalar::all(0));
    for (int i = 0; i < phy.rows; i++)
    {
        float *pr = phy.ptr<float>(i);
        for (int j = 0; j < phy.cols; j++)
        {
            pr[j] = pow(chain[i].x, j);
        }
        y.at<float>(i) = chain[i].y;
    }
    Mat phy_t = phy.t();
    Mat phyMULphy_t = phy.t() * phy;
    Mat phyMphyInv = phyMULphy_t.inv();
    a = phyMphyInv * phy_t;
    a = a * y;
}

/***
 * @description: set mouse callback function to fselect perspective area
 * @param [int] event: mouse event
 * @param [int] x: mouve click pos x
 * @param [int] y: mouve click pos x
 * @param [int] flags:
 * @param [void] *params:
 * @return []
 */
void onMouse(int event, int x, int y, int flags, void *params)
{
    if (event == cv::EVENT_LBUTTONDOWN)
    {
        if (param.flag == 0)
        {
            if (param.var > 0 && param.var % 2 == 1)
            {
                Mat temp = param.show.clone();
                cv::line(temp, Point(0, param.pts[param.var - 1].y), Point(640, param.pts[param.var - 1].y), Scalar(0, 255, 255), 1, 8, 0);
                cv::imshow("Pick four corner points to perspective", temp);
            }
        }
    }
    if (event == EVENT_LBUTTONUP)
    {
        if (param.flag == 0)
        {
            if (param.var == 0)
                param.show = param.src.clone();
            Point point = Point(x, y);
            circle(param.show, point, 2, Scalar(0, 0, 255), -1, 8, 0);
            param.pts.push_back(point);
            param.var++;
            if (param.var > 1)
                cv::line(param.show, param.pts[param.var - 2], point, Scalar(0, 0, 255), 2, 8, 0);

            cv::imshow("Pick four corner points to perspective", param.show);
        }
        cv::imshow("Pick four corner points to perspective", param.show);
    }

    if (event == EVENT_RBUTTONDOWN)
    {
        param.flag = 1;
        param.show = param.src.clone();
        if (param.var != 0)
        {
            cv::polylines(param.show, param.pts, 1, Scalar(0, 0, 0), 2, 8, 0);
        }

        cv::imshow("Pick four corner points to perspective", param.show);
    }
    if (event == EVENT_RBUTTONUP)
    {
        param.flag = param.var;
        Mat final, mask, perspective;
        vector<Point2f> src;
        final = Mat::zeros(param.src.size(), CV_8UC3);
        mask = Mat::zeros(param.src.size(), CV_8UC1);
        cv::fillPoly(mask, param.pts, Scalar(255, 255, 255), 8, 0);
        cv::bitwise_and(param.src, param.src, final, mask);
        cv::imshow("Picked area", final);
        for (Point point : param.pts)
        {
            src.push_back(Point2f(point.x, point.y));
        }
        vector<Point2f> dst = {Point2f(0, 275), Point2f(250, 275), Point2f(250, 125), Point2f(0, 125)};
        param.matrix = getPerspectiveTransform(src, dst);
        cv::warpPerspective(param.src, perspective, param.matrix, Size(500, 300));
        cv::imshow("Perspective", perspective);
        cv::moveWindow("Perspective", 840 + 640, 400);
    }
    if (event == EVENT_MBUTTONDOWN)
    {
        param.pts.clear();
        param.var = 0;
        param.flag = 0;
        cv::imshow("Pick four corner points to perspective", param.src);
    }
}

/***
 * @description: a simple UI, click to select and map the perspective matrix
 * @param [Mat] &originImage: raw image
 * @return []
 */
void ClassicLaneDetect::getPerspectiveMatrix(Mat &originImage)
{
    param.src = originImage.clone();
    cv::namedWindow("Pick four corner points to perspective");
    cv::namedWindow("Picked area");
    cv::moveWindow("Pick four corner points to perspective", 200, 400);
    cv::moveWindow("Picked area", 840, 400);
    cv::imshow("Pick four corner points to perspective", originImage);
    cv::imshow("Picked area", originImage);
    cv::setMouseCallback("Pick four corner points to perspective", onMouse);
    cout << "===============================User Guide==============================\n";
    cout << "1. Left button up to select a point\n"
         << "2. After 4 points picked, press right button to check area\n";
    cout << "3. If not satisfy with picked area, press middle-wheel button to repick\n"
         << "4. If satisfied, press any key to exit\n";
    cout << "!!! Pleae use the YELLOW GUIDE LINE to make top and bootom edges horizontal !!!\n";
    cout << "======================================================================\n";
    cv::waitKey();
    cv::destroyAllWindows();
    warpPerspectiveMatrix = param.matrix.clone();
}

/***
 * @description: light compensation
 * @param [Mat] &originImage: grey image
 * @param [Mat] &compensateImage: compensated grey image
 * @return []
 */
void ClassicLaneDetect::lightCompensate(Mat &originImage, Mat &compensateImage)
{
    originImage.convertTo(originImage, CV_64F, 1);
    double average = mean(originImage)[0];
    double maxVal, minVal;
    int maxInd, minInd;
    int row = processParameters.height / processParameters.blockSize;
    int col = originImage.size[1] / processParameters.blockSize;
    Mat light(Size(col, row), CV_64F);
    for (int i = 0; i < row; i++)
    {
        for (int j = 0; j < col; j++)
        {
            int y = i * processParameters.blockSize;
            int x = j * processParameters.blockSize;
            Rect rect(x, y, processParameters.blockSize, processParameters.blockSize);
            light.at<double>(i, j) = mean(originImage(rect))[0];
        }
    }
    cv::minMaxIdx(light, &minVal, &maxVal, &minInd, &maxInd);
    light = light - average;
    cv::resize(light, light, Size(), processParameters.blockSize, processParameters.blockSize, INTER_CUBIC);
    originImage.convertTo(originImage, CV_8U, 1);
    cv::medianBlur(originImage, originImage, 5);
    cv::addWeighted(originImage, 1.1, light, -0.6, 0, compensateImage, CV_64F);
    originImage.convertTo(originImage, CV_8U, 1);
}

/***
 * @description: image enhancement using gamma-transform and Gaussian blur
 * @param [Mat] &originImage:
 * @param [Mat] &enhancedImage:
 * @return []
 */
void ClassicLaneDetect::imageEnhancement(Mat &originImage, Mat &enhancedImage)
{
    Mat imageGauss, imageGamma;
    originImage = originImage / 125;
    cv::pow(originImage, processParameters.gamma, imageGamma);
    cv::normalize(imageGamma, imageGamma, 255, 0);
    cv::GaussianBlur(imageGamma, imageGauss, Size(3, 3), 5);
    imageGauss.convertTo(enhancedImage, CV_8U, 255);
}

/***
 * @description: get useable grey image
 * @param [Mat] &originImage: raw image
 * @param [Mat] &processImage: grey image where lanes are salient
 * @return []
 */
void ClassicLaneDetect::preProcess(Mat &originImage, Mat &processImage)
{
    Mat perspective, hls, gray, h, l, s;
    Mat imageCompensate;
    vector<Mat> HLS;
    // getPerspectiveMatrix(img);
    // warpPerspective(originImage, perspective, warpPerspectiveMatrix, Size(processParameters.width, processParameters.height));

    Rect rect(0, 480 - processParameters.height, 640, processParameters.height);
    cv::cvtColor(originImage(rect), hls, COLOR_BGR2HLS);
    cv::split(hls, HLS);

    lightCompensate(HLS[1], imageCompensate);
    imageEnhancement(imageCompensate, processImage);
}

/***
 * @description: detect lanes using canny and Hough
 * @param [Mat] &image: grey image
 * @param [vector<Point>] &leftPoints1: left edge points of left lane
 * @param [vector<Point>] &leftPoints2: right edge points of left lane
 * @param [vector<Point>] &rightPoints1: left edge points of right lane
 * @param [vector<Point>] &rightPoints2: right edge points of right lane
 * @return []
 */
void ClassicLaneDetect::detectLaneByEdges(Mat &image, vector<Point> &leftPoints1, vector<Point> &leftPoints2, vector<Point> &rightPoints1, vector<Point> &rightPoints2)
{
    Mat imageCanny, mask;
    int cnt_l1 = 0;
    int cnt_l2 = 0;
    int cnt_r1 = 0;
    int cnt_r2 = 0;
    double left_points_b_mean_1 = 0;
    double left_points_b_mean_2 = 0;
    double right_points_b_mean_1 = 0;
    double right_points_b_mean_2 = 0;
    vector<Vec4f> lines;
    cv::Canny(image, imageCanny, 150, 150);
    mask = Mat::zeros(imageCanny.size(), CV_8UC1);
    cv::fillPoly(mask, processParameters.initROI, Scalar(255, 255, 255), 8, 0);
    cv::bitwise_and(imageCanny, mask, imageCanny);
    cv::HoughLinesP(imageCanny, lines, 1, CV_PI / 180, 25, 5, 15);
    cv::cvtColor(imageCanny, imageCanny, COLOR_GRAY2BGR);
    for (Vec4f lane : lines)
    {
        int x1 = lane[0];
        int y1 = lane[1] + 480 - processParameters.height;
        int x2 = lane[2];
        int y2 = lane[3] + 480 - processParameters.height;
        cv::line(imageCanny, Point(lane[0], lane[1]), Point(lane[2], lane[3]), Scalar(0, 0, 255), 1);
        if (abs(y1 - y2) < 5)
            continue;
        double d = (double)(y1 - y2) / (double)(x1 - x2);
        double b = y1 - d * x1;
        if (d < 0)
        {
            if (cnt_l1 == 0 || abs(b - left_points_b_mean_1) < processParameters.leftClassifyMargin)
            {
                leftPoints1.push_back(Point(x1, y1));
                leftPoints1.push_back(Point(x2, y2));
                left_points_b_mean_1 = (left_points_b_mean_1 * cnt_l1 + b) / (cnt_l1 + 1);
                cnt_l1 = cnt_l1 + 1;
            }
            else if (cnt_l2 == 0 || abs(b - left_points_b_mean_2) < processParameters.leftClassifyMargin)
            {
                leftPoints2.push_back(Point(x1, y1));
                leftPoints2.push_back(Point(x2, y2));
                left_points_b_mean_2 = (left_points_b_mean_2 * cnt_l2 + b) / (cnt_l2 + 1);
                cnt_l2 = cnt_l2 + 1;
            }
        }
        else
        {
            if (cnt_r1 == 0 || abs(b - right_points_b_mean_1) < processParameters.rightClassifyMargin)
            {
                rightPoints1.push_back(Point(x1, y1));
                rightPoints1.push_back(Point(x2, y2));
                right_points_b_mean_1 = (right_points_b_mean_1 * cnt_r1 + b) / (cnt_r1 + 1);
                cnt_r1 = cnt_r1 + 1;
            }
            else if (cnt_r2 == 0 || abs(b - right_points_b_mean_2) < processParameters.rightClassifyMargin)
            {
                rightPoints2.push_back(Point(x1, y1));
                rightPoints2.push_back(Point(x2, y2));
                right_points_b_mean_2 = (right_points_b_mean_2 * cnt_r2 + b) / (cnt_r2 + 1);
                cnt_r2 = cnt_r2 + 1;
            }
        }
    }
}

/***
 * @description: fitorder function by points
 * @param [vector<Point>] &leftPoints1: left edge points of left lane
 * @param [vector<Point>] &leftPoints2: right edge points of left lane
 * @param [vector<Point>] &rightPoints1: left edge points of right lane
 * @param [vector<Point>] &rightPoints2: right edge points of right lane
 * @param [Mat] &leftFunc: fitorder func parameters for leftlane
 * @param [Mat] &rightFunc: fitorder func parameters for rightlane
 * @return []
 */
void ClassicLaneDetect::generateFinalLanes(vector<Point> &leftPoints1, vector<Point> &leftPoints2, vector<Point> &rightPoints1, vector<Point> &rightPoints2, Mat &leftFunc, Mat &rightFunc)
{
    vector<Point> leftLane, rightLane;
    leftLane.insert(leftLane.end(), leftPoints1.begin(), leftPoints1.end());
    leftLane.insert(leftLane.end(), leftPoints2.begin(), leftPoints2.end());
    rightLane.insert(rightLane.end(), rightPoints1.begin(), rightPoints1.end());
    rightLane.insert(rightLane.end(), rightPoints2.begin(), rightPoints2.end());
    if (leftLane.size() != 0)
    {
        polyFit(leftLane, fitOrder + 1, leftFunc);
    }
    else
    {
        Mat emptyMat;
        leftFunc = emptyMat;
    }
    if (rightLane.size() != 0)
    {
        polyFit(rightLane, fitOrder + 1, rightFunc);
    }
    else
    {
        Mat emptyMat;
        rightFunc = emptyMat;
    }
}

/***
 * @description: drow lane on showImage
 * @param [Mat] &showImage:
 * @param [Mat] &laneFunc:
 * @return []
 */
void ClassicLaneDetect::drawLane(Mat &showImage, Mat &laneFunc)
{
    Point downNode, upNode;
    double d = laneFunc.at<float>(0, 1);
    double b = laneFunc.at<float>(0, 0);
    double px = d > 0 ? 640 : 0;
    downNode.y = (d * px + b < 480) ? int(d * px + b) : 480;
    downNode.x = (d * px + b < 480) ? int(px) : int((480 - b) / d);
    upNode.y = 480 - processParameters.height;
    upNode.x = int((upNode.y - b) / d);
    cout << "d: " << d << " b: " << b << endl;
    cout << downNode.y << " " << downNode.x << " " << upNode.y << " " << upNode.x;
    cv::line(showImage, downNode, upNode, Scalar(255, 0, 255), 2);
}

/***
 * @description: draw fitted two lanes on image and display
 * @return []
 */
void ClassicLaneDetect::displayLanes()
{
    Mat showImage = image.clone();
    if (leftFunc.size[0] == fitOrder + 1)
    {
        drawLane(showImage, leftFunc);
    }
    if (rightFunc.size[0] == fitOrder + 1)
    {
        drawLane(showImage, rightFunc);
    }
    cv::imshow("lane", showImage);
    cv::waitKey();
}

void ClassicLaneDetect::run()
{
    // receive image from zmq
    Mat process;
    preProcess(image, process);
    vector<Point> rightLane1, rightLane2, leftLane1, leftLane2;
    detectLaneByEdges(process, leftLane1, leftLane2, rightLane1, rightLane2);
    generateFinalLanes(leftLane1, leftLane2, rightLane1, rightLane2, leftFunc, rightFunc);
    cout << "leftFunc \n"
         << format(leftFunc, Formatter::FMT_DEFAULT) << endl;
    cout << "rightFunc\n"
         << format(rightFunc, Formatter::FMT_DEFAULT) << endl;
    // send leftFunc and rightFunc to zmq
    moduleSelfCheck();
    displayLanes();
}

void ClassicLaneDetect::moduleSelfCheck()
{
    if (leftFunc.size[0] == fitOrder + 1)
    {
        cout << "Left lane funtion satisfied." << endl;
    }
    if (rightFunc.size[0] == fitOrder + 1)
    {
        cout << "Right lane funtion satisfied." << endl;
    }
}

void ClassicLaneDetect::moduleSelfCheckPrint()
{
    cout << "======="
         << "ClassicLaneDetect Parameters"
         << "======";
    cout << "Image size: " << imageSize[0] << "x " << imageSize[1] << endl;
    cout << "Resize height: " << processParameters.height << endl;
    cout << "Light compensate block size: " << processParameters.blockSize << endl;
    cout << "Gamma Transform: " << processParameters.gamma << endl;
}

ClassicLaneDetect::~ClassicLaneDetect()
{
}
