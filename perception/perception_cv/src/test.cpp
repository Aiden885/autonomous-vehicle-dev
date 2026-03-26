#include "img_process.hpp"
#include <iostream>

using namespace imageProcess;

int main()
{
    cv::Mat img;
    cv::Mat output;
    cv::Mat gray;
    std::vector<std::vector<cv::Point>> contours;
    loadImage("/home/ran/Pictures/caltech_lane.png", img);
    imGray(img, gray);

    imDoubleThresholding(gray, output);

    imGetContours(output, img, contours);

    cv::imshow("Merged Images", img);
    cv::waitKey(0);
    cv::destroyAllWindows();
    saveImage(output, "/home/ran/Pictures/test.png");
    return 0;
}



// #include <opencv2/opencv.hpp>
// #include <opencv2/features2d.hpp>
// // #include <opencv2/xfeatures2d.hpp>
// #include <iostream>

// using namespace std;
// using namespace cv;
// // using namespace cv::xfeatures2d;

// void showImg(const char* w_name, const cv::Mat& img, int flg = 0)
// {
//     cv::namedWindow(w_name, cv::WINDOW_NORMAL);
//     cv::imshow(w_name, img);
//     if (flg == 1)
//     {
//         cv::waitKey(0);
//     }
//     else if (flg > 1)
//     {
//         cv::waitKey(flg);
//     }
// }

// Mat stichingWithSURF(Mat mat1, Mat mat2);
// void calCorners(const Mat& H, const Mat& src);//计算变换后的角点
// Mat extractFeatureAndMatch(Mat mat1, Mat mat2); //特征提取和匹配
// Mat splicImg(Mat& mat1, Mat& mat2, vector<DMatch> goodMatchPoints, vector<KeyPoint> keyPoint1, vector<KeyPoint> keyPoint2);

// void stichingWithStitcher(Mat mat1, Mat mat2);

// typedef struct
// {
//     //变换后的图片4个点
//     Point2f left_top;
//     Point2f left_bottom;
//     Point2f right_top;
//     Point2f right_bottom;
// }four_corners_t;
// four_corners_t corners;


// void main()
// {
//     Mat img1, img2;
//     img1 = imread("./imgs/1.jpg");
//     img2 = imread("./imgs/2.jpg");
//     resize(img1, img1, Size(img1.cols / 4, img1.rows / 4));
//     resize(img2, img2, Size(img2.cols / 4, img2.rows / 4));

//     Mat dst = stichingWithSURF(img1, img2);

//     showImg("拼好的图像", dst);
//     imwrite("result1.jpg", dst);

//     waitKey();
// }

// Mat stichingWithSURF(Mat mat1, Mat mat2)
// {
//     //用SURF 是因为 SURF有旋转不变性而且比SIFT更快 
//     /*
//         1.特征点提取和匹配
//         2.图像配准
//         3.图像拷贝
//         4.图像融合

//     */
//     return extractFeatureAndMatch(mat1, mat2);

// }

// //定位图像变换之后的四个角点
// void calCorners(const Mat& H, const Mat& src)
// {
//     //H为 变换矩阵  src为需要变换的图像

//     //计算配准图的角点（齐次坐标系描述）
//     double v2[] = { 0,0,1 }; //左上角
//     double v1[3];  //变换后的坐标值
//     //构成 列向量` 这种构成方式将 向量 与 Mat 关联， Mat修改 向量也相应修改
//     Mat V2 = Mat(3, 1, CV_64FC1, v2);
//     Mat V1 = Mat(3, 1, CV_64FC1, v1);
//     V1 = H * V2; //元素* 
//     cout << "0v1:" << v1[0] << endl;
//     cout << "V2: " << V2 << endl;
//     cout << "V1: " << V1 << endl;

//     //左上角（转换为一般的二维坐标系）
//     corners.left_top.x = v1[0] / v1[2];
//     corners.left_top.y = v1[1] / v1[2];

//     //左下角（0，src.rows，1）
//     v2[0] = 0;
//     v2[1] = src.rows;
//     v2[2] = 1;
//     V2 = Mat(3, 1, CV_64FC1, v2);
//     V1 = Mat(3, 1, CV_64FC1, v1);  //列向量
//     V1 = H * V2;
//     cout << "1v1:" << v1[0] << endl;
//     cout << "V2: " << V2 << endl;
//     cout << "V1: " << V1 << endl;
//     corners.left_bottom.x = v1[0] / v1[2];
//     corners.left_bottom.y = v1[1] / v1[2];

//     //右上角（src.cols,0,1)
//     v2[0] = src.cols;
//     v2[1] = 0;
//     v2[2] = 1;
//     V2 = Mat(3, 1, CV_64FC1, v2);
//     V1 = Mat(3, 1, CV_64FC1, v1);
//     V1 = H * V2;
//     cout << "2v1:" << v1 << endl;
//     cout << "V2: " << V2 << endl;
//     cout << "V1: " << V1 << endl;
//     corners.right_top.x = v1[0] / v1[2];
//     corners.right_top.y = v1[1] / v1[2];

//     //右下角(src.cols,src.rows,1)
//     v2[0] = src.cols;
//     v2[1] = src.rows;
//     v2[2] = 1;
//     V2 = Mat(3, 1, CV_64FC1, v2);  //列向量
//     V1 = Mat(3, 1, CV_64FC1, v1);  //列向量
//     V1 = H * V2;
//     cout << "3v1:" << v1 << endl;
//     cout << "V2: " << V2 << endl;
//     cout << "V1: " << V1 << endl;

//     corners.right_bottom.x = v1[0] / v1[2];
//     corners.right_bottom.y = v1[1] / v1[2];

//     cout << endl;
//     cout << "left_top:" << corners.left_top << endl;
//     cout << "left_bottom:" << corners.left_bottom << endl;
//     cout << "right_top:" << corners.right_top << endl;
//     cout << "right_bottom:" << corners.right_bottom << endl;
// }

// Mat extractFeatureAndMatch(Mat mat1, Mat mat2)
// {
//     Mat matg1, matg2;
//     //转化成灰度图
//     cvtColor(mat1, matg1, COLOR_BGR2GRAY);
//     cvtColor(mat2, matg2, COLOR_BGR2GRAY);


//     Ptr<SURF> surfDetector = SURF::create(1000);
//     vector<KeyPoint> keyPoint1, keyPoint2; //特征点
//     Mat imgDesc1, imgDesc2; //特征点描述矩阵
//     //检测 计算图像的关键点和描述
//     surfDetector->detectAndCompute(matg1, noArray(), keyPoint1, imgDesc1);
//     surfDetector->detectAndCompute(matg2, noArray(), keyPoint2, imgDesc2);

//     cout << "特征点描述矩阵1大小:(列*行) " << imgDesc1.cols << " * " << imgDesc1.rows << endl;

//     FlannBasedMatcher matcher;  //匹配点
//     vector<vector<DMatch>> matchPoints; //
//     vector<DMatch> goodMatchPoints; //良好的匹配点


//     //knn匹配特征点 这里将2作为训练集来训练 对应到后面DMatch的trainIdx
//     vector<Mat> train_disc(1, imgDesc2);
//     matcher.add(train_disc);
//     matcher.train();
//     //用1来匹配该模型（用分类器去分类1），对应到后面DMatch的quiryIdx
//     matcher.knnMatch(imgDesc1, matchPoints, 2);//k临近 按顺序排
//     cout << "total match points: " << matchPoints.size() << endl;

//     /*
//     查找集（Query Set）和训练集（Train Set），
//     对于每个Query descriptor，DMatch中保存了和其最好匹配的Train descriptor。
//     */

//     //获取优秀匹配点
//     for (int i = 0; i < matchPoints.size(); i++)
//     {
//         if (matchPoints[i][0].distance < 0.4f * matchPoints[i][1].distance)
//         {
//             goodMatchPoints.push_back(matchPoints[i][0]);
//         }
//     }

//     Mat firstMatch;
//     //这里drawMatches 第一个图片在左边，同时也对应了DMatch的quiryIdx，第二个图片在右边，同时也对应了DMatch的trainIdx
//     drawMatches(mat1, keyPoint1, mat2, keyPoint2, goodMatchPoints, firstMatch);

//     showImg("匹配", firstMatch);
//     imwrite("match1.jpg", firstMatch);

//     vector<Point2f> imagePoints1, imagePoints2;
//     for (int i = 0; i < goodMatchPoints.size(); i++)
//     {
//         imagePoints1.push_back(keyPoint1[goodMatchPoints[i].queryIdx].pt);
//         imagePoints2.push_back(keyPoint2[goodMatchPoints[i].trainIdx].pt);
//     }

//     return splicImg(mat1, mat2, goodMatchPoints, keyPoint1, keyPoint2);

// }

// //以图像1为准（1在左半边）
// Mat splicImg(Mat& mat_left, Mat& mat2, vector<DMatch> goodMatchPoints, vector<KeyPoint> keyPoint1, vector<KeyPoint> keyPoint2)
// {
//     vector<Point2f> imagePoints1, imagePoints2;
//     for (int i = 0; i < goodMatchPoints.size(); i++)
//     {
//         //这里的queryIdx代表了查询点的目录     trainIdx代表了在匹配时训练分类器所用的点的目录
//         imagePoints1.push_back(keyPoint1[goodMatchPoints[i].queryIdx].pt);
//         imagePoints2.push_back(keyPoint2[goodMatchPoints[i].trainIdx].pt);
//     }

//     //获取图像2到图像1的投影映射矩阵 3*3
//     Mat homo = findHomography(imagePoints2, imagePoints1, RANSAC);
//     cout << "变换矩阵为：\n" << homo << endl << endl; //输出映射矩阵  

//     calCorners(homo, mat2); //计算配准图的四个顶点坐标
//     Mat imgTransform2;

//     //图像配准 warpPerspective 对图像进行透视变换 变换后矩阵的宽高都变化
//     warpPerspective(mat2, imgTransform2, homo, 
//         Size(MAX(corners.right_top.x, corners.right_bottom.x), MAX(corners.left_bottom.y, corners.right_bottom.y)));
//     showImg("直接经过透视矩阵变换得到的img2", imgTransform2);

//     //创建拼接后的图
//     //int distW = imgTransform2.cols; //长宽
//     //int distH = mat_left.rows;
//     Mat dst(imgTransform2.size(), CV_8UC3);
//     dst.setTo(0);

//     //构成图片  
//     //复制img2到dist的右半部分 先复制transform2的图片（因为这个尺寸比较大，后来的图片可以覆盖到他）
//     imgTransform2.copyTo(dst(Rect(0, 0, imgTransform2.cols, imgTransform2.rows)));
//     mat_left.copyTo(dst(Rect(0, 0, mat_left.cols, mat_left.rows)));

//     return dst;
// }

