/**
 * @brief 坐标转换头文件
 * @file coordinateTransFunctional.h
 * @version 0.0.1
 * @author Hongyu Liu
 * @date 2023-12-18
 */
#include "ztgeographycoordinatetransform.h"
/**
 * \namespace Localization
 * \brief Localization namespace
 */

namespace Localization{

struct XY2BLParam
{
    ZtGeographyCoordinateTransform Zt;
};
struct XY2BLInput
{
    double xInput;
    double yInput;
};
struct XY2BLOutput
{
    double latOutput;
    double lonOutput;
};

struct BL2XYParam
{
    ZtGeographyCoordinateTransform Zt;
};
struct BL2XYInput
{
    double latInput;
    double lonInput;
};
struct BL2XYOutput
{
    double xOutput;
    double yOutput;
};

struct XYZ2BLHParam
{
    ZtGeographyCoordinateTransform Zt;
};
struct XYZ2BLHInput
{
    double xInput;
    double yInput;
    double zInput;
};
struct XYZ2BLHOutput
{
    double latOutput;
    double lonOutput;
    double highOutput;
};

struct BLH2XYZParam
{
    ZtGeographyCoordinateTransform Zt;
};
struct BLH2XYZInput
{
    double latInput;
    double lonInput;
    double highInput;
};
struct BLH2XYZOutput
{
    double xOutput;
    double yOutput;
    double zOutput;
};

struct V2BLHParam
{
    ZtGeographyCoordinateTransform Zt;
};
struct V2BLHInput
{
    double x0Input, y0Input, z0Input, x_vInput, y_vInput, z_vInput, thetaInput;
};
struct V2BLHOutput
{
    double x_wOutput, y_wOutput, z_wOutput;
};

struct BLH2VParam
{
    ZtGeographyCoordinateTransform Zt;
};
struct BLH2VInput
{
    double x0Input, y0Input, z0Input, x_wInput, y_wInput, z_wInput, thetaInput;
};
struct BLH2VOutput
{
    double x_vOutput, y_vOutput, z_vOutput;
};
/**
 * @brief ENU转LLA(二维)
 * @param[IN] param 坐标转换类对象
 * @param[IN] input ENU坐标(x,y)
 * @param[IN] output LLA坐标
 * @cn_name: ENU转LLA(二维)
 * @granularity: atomic //函数组件的粒度， atomic： 基础组件（原子），composite： 复合组件（在前端可以向下一层展开）
 * @tag: localization
 */
void ztGeographyCoordinateTransformXY2BL(XY2BLParam &param01, XY2BLInput &input01, XY2BLOutput &output01);
/**
 * @brief LLA转ENU（二维）
 * @param[IN] param 坐标转换类对象
 * @param[IN] input LLA坐标
 * @param[IN] output ENU坐标(x,y)
 * @cn_name: LLA转ENU（二维）
 * @granularity: atomic //函数组件的粒度， atomic： 基础组件（原子），composite： 复合组件（在前端可以向下一层展开）
 * @tag: localization
 */
void ztGeographyCoordinateTransformBL2XY(BL2XYParam &param02, BL2XYInput &input02, BL2XYOutput &output02);
/**
 * @brief ENU转LLA(三维)
 * @param[IN] param 坐标转换类对象
 * @param[IN] input ENU坐标(x,y，z)
 * @param[IN] output LLA坐标
 * @cn_name: ENU转LLA(三维)
 * @granularity: atomic //函数组件的粒度， atomic： 基础组件（原子），composite： 复合组件（在前端可以向下一层展开）
 * @tag: localization
 */
void ztGeographyCoordinateTransformXYZ2BLH(XYZ2BLHParam &param03, XYZ2BLHInput &input03, XYZ2BLHOutput &output03);
/**
 * @brief LLA转ENU（三维）
 * @param[IN] param 坐标转换类对象
 * @param[IN] input LLA坐标
 * @param[IN] output ENU坐标
 * @cn_name: LLA转ENU（三维）
 * @granularity: atomic //函数组件的粒度， atomic： 基础组件（原子），composite： 复合组件（在前端可以向下一层展开）
 * @tag: localization
 */
void ztGeographyCoordinateTransformBLH2XYZ(BLH2XYZParam &param04, BLH2XYZInput &input04, BLH2XYZOutput &output04);
/**
 * @brief 全局坐标ENU
 * @param[IN] param 坐标转换类对象
 * @param[IN] input 车辆在ENU下坐标（x0，y0，z0），传感器相对于车身的坐标(x_v,y_v,z_v)，车辆前进方向与正北的夹角theta
 * @param[IN] output 全局坐标(x_w,y_w,z_w)
 * @cn_name: 全局坐标ENU
 * @granularity: atomic //函数组件的粒度， atomic： 基础组件（原子），composite： 复合组件（在前端可以向下一层展开）
 * @tag: localization
 */
void ztGeographyCoordinateTransformV2BLH(V2BLHParam &param05, V2BLHInput &input05, V2BLHOutput &output05);
/**
 * @brief 全局坐标系（ENU）转车辆坐标系
 * @param[IN] param 坐标转换类对象
 * @param[IN] input 车辆在ENU下坐标（x0，y0，z0），全局坐标(x_w,y_w,z_w)，车辆前进方向与正北的夹角theta
 * @param[IN] output 传感器相对于车身的坐标(x_v,y_v,z_v)
 * @cn_name: 全局坐标系（ENU）转车辆坐标系
 * @granularity: atomic //函数组件的粒度， atomic： 基础组件（原子），composite： 复合组件（在前端可以向下一层展开）
 * @tag: localization
 */
void ztGeographyCoordinateTransformBLH2V(BLH2VParam &param06, BLH2VInput &input06, BLH2VOutput &output06);
}