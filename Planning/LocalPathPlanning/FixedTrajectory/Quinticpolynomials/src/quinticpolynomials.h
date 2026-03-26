#include <vector>
#include <stdint.h>
#include <cmath>
#include <Eigen/Cholesky>
#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/SVD>
#include <iostream>

/**
 * @brief 五次多项式生成相关声明
 * @file quinticpolynomials.h
 * @version 0.0.1
 * @author Zehang Zhu (zzh22@mails.tsinghua.edu.cn)
 * @date 2023-11-25
 */

#define CURVE_POINT_NUM 50

/**
 * \namespace planning
 * \brief planning namespace
 */

namespace planning{

struct Point
{
  double x;
  double y;
  double angle;
};

struct GaussRoadPoint {
	double GaussX;
	double GaussY;
	double yaw;
	double curvature;
};

struct Curve
{
  Point points[CURVE_POINT_NUM];
};

struct GQPPParam{
};

struct GQPPInput{
	GaussRoadPoint startPoint;
	GaussRoadPoint endPoint;
	Curve path;
};

struct GQPPOutput{
	Curve path;
};

struct PQPParam{
};

struct PQPInput{
	std::vector<double> coeff;
	double t;
	double x0;
	double x1;
};

struct PQPOutput{
	Point point;
};


void generateQuinticPolynomialsPath(const GQPPParam &param, const GQPPInput &input, GQPPOutput &output);

void pointOnQuinticPolynomials(const PQPParam &param, const PQPInput &input, PQPOutput &output);

}