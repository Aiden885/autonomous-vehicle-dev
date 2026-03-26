/**
 * @brief 五次多项式轨迹生成相关函数
 * @file quinticpolynomials.cpp
 * @version 0.0.1
 * @author Zehang Zhu (zzh22@mails.tsinghua.edu.cn)
 * @date 2023-11-25
 */

#include "quinticpolynomials.h"

namespace planning{

/**
 * @brief 生成五次多项式轨迹
 * @param[IN] param 无
 * @param[IN] input 起始点，终止点，初始化的路径
 * @param[OUT] output 生成的路径
 
 * @cn_name: 生成五次多项式轨迹
 
 * @granularity: composite //函数组件的粒度， atomic： 基础组件（原子），composite： 复合组件（在前端可以向下一层展开）
 
 * @tag: planning
 */
void generateQuinticPolynomialsPath(const GQPPParam &param, const GQPPInput &input, GQPPOutput &output)
{
  GaussRoadPoint startPoint = input.startPoint;
  GaussRoadPoint endPoint = input.endPoint;
  Curve path = input.path;

  double dx = endPoint.GaussX - startPoint.GaussX;
  double dy = endPoint.GaussY - startPoint.GaussY;

  Point spoint;
  spoint.x = 0.0;
  spoint.y = 0.0;
  spoint.angle = startPoint.yaw;

  Point epoint;
  epoint.x = dx;
  epoint.y = dy;
  epoint.angle = endPoint.yaw;

  double x0 = spoint.x;
  double x1 = epoint.x;

  Eigen::Matrix< double, 6, 6> A1;
  A1 << 1, x0, pow(x0,2), pow(x0,3), pow(x0,4), pow(x0,5),
        0, 1, 2*x0, 3*pow(x0,2), 4*pow(x0,3), 5*pow(x0,4),
        0, 0, 2, 6*x0, 12*pow(x0,2), 20*pow(x0,3),
        1, x1, pow(x1,2), pow(x1,3), pow(x1,4), pow(x1,5),
        0, 1, 2*x1, 3*pow(x1,2), 4*pow(x1,3), 5*pow(x1,4),
        0, 0, 2, 6*x1, 12*pow(x1,2), 20*pow(x1,3);

  Eigen::Matrix< double, 6, 1> b1;
  b1 << spoint.y,
        spoint.angle,
        0,
        epoint.y,
        epoint.angle,
        0;

  Eigen::Matrix<double, 6, 1> ans;
  ans = A1.householderQr().solve(b1);
  std::cout << ans << std::endl;
  std::vector<double> coeff;
  coeff.push_back(ans(0,0)); 
  coeff.push_back(ans(1,0)); 
  coeff.push_back(ans(2,0)); 
  coeff.push_back(ans(3,0));
  coeff.push_back(ans(4,0));
  coeff.push_back(ans(5,0)); 

  double dt = 1.0 / (CURVE_POINT_NUM - 1);
  for (uint32_t i = 0; i < CURVE_POINT_NUM; i++)
  {
    Point resultpoint;

    PQPParam param0{};
    PQPInput input0{coeff, i * dt, x0, x1};
    PQPOutput output0{resultpoint};

    pointOnQuinticPolynomials(param0, input0, output0);

    resultpoint = output0.point;
    path.points[i].x = resultpoint.x + startPoint.GaussX;
    path.points[i].y = resultpoint.y + startPoint.GaussY;
    path.points[i].angle = static_cast<int32_t>(360 + resultpoint.angle + startPoint.yaw) % 360;
  }

  output.path = path;
 }

/**
 * @brief 五次多项式轨迹点计算
 * @param[IN] param 无
 * @param[IN] input 五次多项式系数，轨迹点参数，起点，终点
 * @param[OUT] output 生成轨迹点
 
 * @cn_name: 计算五次多项式轨迹上的点
 
 * @granularity: atomic //函数组件的粒度， atomic： 基础组件（原子），composite： 复合组件（在前端可以向下一层展开）
 
 * @tag: planning
 */
void pointOnQuinticPolynomials(const PQPParam &param, const PQPInput &input, PQPOutput &output)
{
    std::vector<double> coeff = input.coeff;
    double t = input.t;
    double x0 = input.x0;
    double x1 = input.x1;

    Point result;
    result.x = x0 + t*(x1 - x0);
    result.y = coeff[0] + coeff[1]*result.x + coeff[2]*pow(result.x,2) + coeff[3]*pow(result.x,3) + coeff[4]*pow(result.x,4) + coeff[5]*pow(result.x,5);

    output.point = result;
    return;
}
}