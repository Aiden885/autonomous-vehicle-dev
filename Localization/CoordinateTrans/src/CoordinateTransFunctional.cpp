/**
 * @brief 坐标转换源文件
 * @file coordinateTransFunctional.cpp
 * @version 0.0.1
 * @author Hongyu Liu
 * @date 2023-12-18
 */
#include <iostream>
#include "coordinateTransFunctional.h"
using namespace std;

namespace Localization{
/**
 * @brief ENU转LLA(二维)
 * @param[IN] param 坐标转换类对象
 * @param[IN] input ENU坐标(x,y)
 * @param[OUT] output LLA坐标
 
 * @cn_name: ENU转LLA(二维)
 
 * @granularity: atomic //函数组件的粒度， atomic： 基础组件（原子），composite： 复合组件（在前端可以向下一层展开）
 
 * @tag: localization
 */
void ztGeographyCoordinateTransformXY2BL(XY2BLParam &param01, XY2BLInput &input01, XY2BLOutput &output01)
{
    ZtGeographyCoordinateTransform Zt = param01.Zt;
    double &xInput = input01.xInput;
    double &yInput = input01.yInput;
    double &latOutput = output01.latOutput;
    double &lonOutput = output01.lonOutput;
    Zt.XY2BL(xInput, yInput, latOutput, lonOutput);
}
/**
 * @brief LLA转ENU（二维）
 * @param[IN] param 坐标转换类对象
 * @param[IN] input LLA坐标
 * @param[OUT] output ENU坐标(x,y)
 
 * @cn_name: LLA转ENU（二维）
 
 * @granularity: atomic //函数组件的粒度， atomic： 基础组件（原子），composite： 复合组件（在前端可以向下一层展开）
 
 * @tag: localization
 */
void ztGeographyCoordinateTransformBL2XY(BL2XYParam &param02, BL2XYInput &input02, BL2XYOutput &output02)
{
    ZtGeographyCoordinateTransform &Zt = param02.Zt;
    double &latInput = input02.latInput;
    double &lonInput = input02.lonInput;
    double &xOutput = output02.xOutput;
    double &yOutput = output02.yOutput;
    Zt.BL2XY(latInput, lonInput, xOutput, yOutput);
}
/**
 * @brief ENU转LLA(三维)
 * @param[IN] param 坐标转换类对象
 * @param[IN] input ENU坐标(x,y，z)
 * @param[OUT] output LLA坐标
 
 * @cn_name: ENU转LLA(三维)
 
 * @granularity: atomic //函数组件的粒度， atomic： 基础组件（原子），composite： 复合组件（在前端可以向下一层展开）
 
 * @tag: localization
 */
void ztGeographyCoordinateTransformXYZ2BLH(XYZ2BLHParam &param03, XYZ2BLHInput &input03, XYZ2BLHOutput &output03)
{
    ZtGeographyCoordinateTransform &Zt = param03.Zt;
    double &xInput = input03.xInput;
    double &yInput = input03.yInput;
    double &zInput = input03.zInput;
    double &latOutput = output03.latOutput;
    double &lonOutput = output03.lonOutput;
    double &highOutput = output03.highOutput;
    Zt.XYZ2BLH(xInput, yInput, zInput, latOutput, lonOutput, highOutput);
}
/**
 * @brief LLA转ENU（三维）
 * @param[IN] param 坐标转换类对象
 * @param[IN] input LLA坐标
 * @param[OUT] output ENU坐标
 
 * @cn_name: LLA转ENU（三维）
 
 * @granularity: atomic //函数组件的粒度， atomic： 基础组件（原子），composite： 复合组件（在前端可以向下一层展开）
 
 * @tag: localization
 */
void ztGeographyCoordinateTransformBLH2XYZ(BLH2XYZParam &param04, BLH2XYZInput &input04, BLH2XYZOutput &output04)
{
    ZtGeographyCoordinateTransform &Zt = param04.Zt;
    double &latInput = input04.latInput;
    double &lonInput = input04.lonInput;
    double &highInput = input04.highInput;
    double &xOutput = output04.xOutput;
    double &yOutput = output04.yOutput;
    double &zOutput = output04.zOutput;
    Zt.BLH2XYZ(latInput, lonInput, highInput, xOutput, yOutput, zOutput);
}
/**
 * @brief 全局坐标ENU
 * @param[IN] param 坐标转换类对象
 * @param[IN] input 车辆在ENU下坐标（x0，y0，z0），传感器相对于车身的坐标(x_v,y_v,z_v)，车辆前进方向与正北的夹角theta
 * @param[OUT] output 全局坐标(x_w,y_w,z_w)
 
 * @cn_name: 全局坐标ENU
 
 * @granularity: atomic //函数组件的粒度， atomic： 基础组件（原子），composite： 复合组件（在前端可以向下一层展开）
 
 * @tag: localization
 */
void ztGeographyCoordinateTransformV2BLH(V2BLHParam &param05, V2BLHInput &input05, V2BLHOutput &output05)
{
    ZtGeographyCoordinateTransform &Zt = param05.Zt;
    double &x0Input = input05.x0Input;
    double &y0Input = input05.y0Input;
    double &z0Input = input05.z0Input;
    double &x_vInput = input05.x_vInput;
    double &y_vInput = input05.y_vInput;
    double &z_vInput = input05.z_vInput;
    double &thetaInput = input05.thetaInput;
    double &x_wOutput = output05.x_wOutput;
    double &y_wOutput = output05.y_wOutput;
    double &z_wOutput = output05.z_wOutput;
    Zt.V2BLH(x0Input, y0Input, z0Input, x_vInput, y_vInput, z_vInput, thetaInput, x_wOutput, y_wOutput, z_wOutput);
}
/**
 * @brief 全局坐标系（ENU）转车辆坐标系
 * @param[IN] param 坐标转换类对象
 * @param[IN] input 车辆在ENU下坐标（x0，y0，z0），全局坐标(x_w,y_w,z_w)，车辆前进方向与正北的夹角theta
 * @param[OUT] output 传感器相对于车身的坐标(x_v,y_v,z_v)
 
 * @cn_name: 全局坐标系（ENU）转车辆坐标系
 
 * @granularity: atomic //函数组件的粒度， atomic： 基础组件（原子），composite： 复合组件（在前端可以向下一层展开）
 
 * @tag: localization
 */
void ztGeographyCoordinateTransformBLH2V(BLH2VParam &param06, BLH2VInput &input06, BLH2VOutput &output06)
{
    ZtGeographyCoordinateTransform &Zt = param06.Zt;
    double &x0Input = input06.x0Input;
    double &y0Input = input06.y0Input;
    double &z0Input = input06.z0Input;
    double &x_wInput = input06.x_wInput;
    double &y_wInput = input06.y_wInput;
    double &z_wInput = input06.z_wInput;
    double &thetaInput = input06.thetaInput;
    double &x_vOutput = output06.x_vOutput;
    double &y_vOutput = output06.y_vOutput;
    double &z_vOutput = output06.z_vOutput;
    Zt.BLH2V(x0Input, y0Input, z0Input, x_wInput, y_wInput, z_wInput, thetaInput, x_vOutput, y_vOutput, z_vOutput);
}

}