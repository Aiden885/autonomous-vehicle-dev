#ifndef LANE_DETECTION_H
#define LANE_DETECTION_H

#include <opencv2/opencv.hpp>
#include <iostream>
#include <string>
#include <algorithm>

using namespace std;

//typedef unsigned char uchar;
const double PI = acos(-1.0); // 圆周率

bool is_similar(pair<int, int> &l1, pair<int, int> &l2);
bool cmp(const pair<pair<int, int>, int> &g1, const pair<pair<int, int>, int> &g2);
void houghTransform(cv::Mat &src, vector<pair<int, int>> &lines, int threshold);

#endif