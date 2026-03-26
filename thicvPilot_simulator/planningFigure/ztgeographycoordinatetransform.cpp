#include "ztgeographycoordinatetransform.h"
#include <iostream>

/*
 *    此处未定义PI，直接使用PI值，防止与其他文件宏定义冲突
 *    theta: 车辆前进方向与正北方向夹角
 */

ZtGeographyCoordinateTransform::ZtGeographyCoordinateTransform()
    : meridianLine(-360), projType('g')
{
}

ZtGeographyCoordinateTransform::~ZtGeographyCoordinateTransform()
{
}

// ENU转LLA(二维)
// 输入：ENU坐标(x,y) ; 输出：LLA坐标(latitude，longitude)
bool ZtGeographyCoordinateTransform::XY2BL(double x_east, double y_north, double &lat, double &lon)
{
    if (projType == 'u')
    {
        y_north = y_north / 0.9996;
    }

    double bf0 = y_north / ellipPmt.a0, bf;
    double threshould = 1.0;
    while (threshould > 0.00000001)
    {
        double y0 = -ellipPmt.a2 * sin(2 * bf0) / 2 + ellipPmt.a4 * sin(4 * bf0) / 4 - ellipPmt.a6 * sin(6 * bf0) / 6;
        bf = (y_north - y0) / ellipPmt.a0;
        threshould = bf - bf0;
        bf0 = bf;
    }

    double t, j2;
    t = tan(bf);
    j2 = ellipPmt.ep2 * pow(cos(bf), 2);

    double v, n, m;
    v = sqrt(1 - ellipPmt.e2 * sin(bf) * sin(bf));
    n = ellipPmt.a / v;
    m = ellipPmt.a * (1 - ellipPmt.e2) / pow(v, 3);

    x_east = x_east - 500000;
    if (projType == 'u')
    {
        x_east = x_east / 0.9996;
    }

    double temp0, temp1, temp2;
    temp0 = t * x_east * x_east / (2 * m * n);
    temp1 = t * (5 + 3 * t * t + j2 - 9 * j2 * t * t) * pow(x_east, 4) / (24 * m * pow(n, 3));
    temp2 = t * (61 + 90 * t * t + 45 * pow(t, 4)) * pow(x_east, 6) / (720 * pow(n, 5) * m);
    lat = (bf - temp0 + temp1 - temp2) * 57.29577951308232;

    temp0 = x_east / (n * cos(bf));
    temp1 = (1 + 2 * t * t + j2) * pow(x_east, 3) / (6 * pow(n, 3) * cos(bf));
    temp2 = (5 + 28 * t * t + 6 * j2 + 24 * pow(t, 4) + 8 * t * t * j2) * pow(x_east, 5) / (120 * pow(n, 5) * cos(bf));
    lon = (temp0 - temp1 + temp2) * 57.29577951308232 + meridianLine;

    return true;
}

// LLA转ENU（二维）
// 输入：LLA坐标（latitude，longitude）; 输出：ENU坐标(x,y)
bool ZtGeographyCoordinateTransform::BL2XY(double lat, double lon, double &x_east, double &y_north)
{
    if (meridianLine < -180)
    {
        meridianLine = int((lon + 1.5) / 3) * 3;
    }

    lat = lat * 0.0174532925199432957692;
    double dL = (lon - meridianLine) * 0.0174532925199432957692;

    double X = ellipPmt.a0 * lat - ellipPmt.a2 * sin(2 * lat) / 2 + ellipPmt.a4 * sin(4 * lat) / 4 - ellipPmt.a6 * sin(6 * lat) / 6;
    double tn = tan(lat);
    double tn2 = tn * tn;
    double tn4 = tn2 * tn2;

    double j2 = (1 / pow(1 - ellipPmt.f, 2) - 1) * pow(cos(lat), 2);
    double n = ellipPmt.a / sqrt(1.0 - ellipPmt.e2 * sin(lat) * sin(lat));

    double temp[6] = {0};
    temp[0] = n * sin(lat) * cos(lat) * dL * dL / 2;
    temp[1] = n * sin(lat) * pow(cos(lat), 3) * (5 - tn2 + 9 * j2 + 4 * j2 * j2) * pow(dL, 4) / 24;
    temp[2] = n * sin(lat) * pow(cos(lat), 5) * (61 - 58 * tn2 + tn4) * pow(dL, 6) / 720;
    temp[3] = n * cos(lat) * dL;
    temp[4] = n * pow(cos(lat), 3) * (1 - tn2 + j2) * pow(dL, 3) / 6;
    temp[5] = n * pow(cos(lat), 5) * (5 - 18 * tn2 + tn4 + 14 * j2 - 58 * tn2 * j2) * pow(dL, 5) / 120;

    y_north = X + temp[0] + temp[1] + temp[2];
    x_east = temp[3] + temp[4] + temp[5];

    if (projType == 'g')
    {
        x_east = x_east + 500000;
    }
    else if (projType == 'u')
    {
        x_east = x_east * 0.9996 + 500000;
        y_north = y_north * 0.9996;
    }

    return true;
}

// ENU转LLA(三维)
// 输入：ENU坐标(x,y，z) ; 输出：LLA坐标(latitude，longitude, hight)
bool ZtGeographyCoordinateTransform::XYZ2BLH(double x, double y, double z, double &lat, double &lon, double &ht)
{
    double preB, preN;
    double nowB = 0, nowN = 0;
    double threshould = 1.0;

    preB = atan(z / sqrt(x * x + y * y));
    preN = ellipPmt.a / sqrt(1 - ellipPmt.e2 * sin(preB) * sin(preB));
    while (threshould > 0.0000000001)
    {
        nowN = ellipPmt.a / sqrt(1 - ellipPmt.e2 * sin(preB) * sin(preB));
        nowB = atan((z + preN * ellipPmt.e2 * sin(preB)) / sqrt(x * x + y * y));

        threshould = fabs(nowB - preB);
        preB = nowB;
        preN = nowN;
    }
    ht = sqrt(x * x + y * y) / cos(nowB) - nowN;
    lon = atan2(y, x) * 57.29577951308232; // 180 / pi
    lat = nowB * 57.29577951308232;

    return true;
}

// LLA转ENU（三维）
// 输入：LLA坐标（latitude，longitude，hight）; 输出：ENU坐标(x,y,z)
bool ZtGeographyCoordinateTransform::BLH2XYZ(double lat, double lon, double ht, double &x, double &y, double &z)
{
    double sinB = sin(lat / 57.29577951308232);
    double cosB = cos(lat / 57.29577951308232);
    double sinL = sin(lon / 57.29577951308232);
    double cosL = cos(lon / 57.29577951308232);

    double N = ellipPmt.a / sqrt(1.0 - ellipPmt.e2 * sinB * sinB);
    x = (N + ht) * cosB * cosL;
    y = (N + ht) * cosB * sinL;
    z = (N * ellipPmt.b * ellipPmt.b / (ellipPmt.a * ellipPmt.a) + ht) * sinB;

    return true;
}

// 全局坐标ENU  x: east, y: north, z: height, theta:车辆前进方向与正北的夹角。

// 车辆坐标系转全局坐标系（ENU）
// 输入：车辆在ENU下坐标（x0，y0，z0），传感器相对于车身的坐标(x_v,y_v,z_v)，车辆前进方向与正北的夹角theta
// 输出：全局坐标(x_w,y_w,z_w)
bool ZtGeographyCoordinateTransform::V2BLH(double x0, double y0, double z0, double x_v, double y_v, double z_v, double theta, double &x_w, double &y_w, double &z_w)
{
    x_w = x_v * cos(theta) - y_v * sin(theta) + x0;
    y_w = x_v * sin(theta) + y_v * cos(theta) + y0;
    z_w = z0;

    return true;
}

// 全局坐标系（ENU）转车辆坐标系
// 输入：车辆在ENU下坐标（x0，y0，z0），全局坐标(x_w,y_w,z_w)，车辆前进方向与正北的夹角theta
// 输出：传感器相对于车身的坐标(x_v,y_v,z_v)
bool ZtGeographyCoordinateTransform::BLH2V(double x0, double y0, double z0, double x_w, double y_w, double z_w, double theta, double &x_v, double &y_v, double &z_v)
{
    x_v = (x_w - x0) * cos(theta) + (y_w - y0) * sin(theta);
    y_v = (y_w - y0) * cos(theta) + (x_w - x0) * sin(theta);
    z_v = z0;

    return true;
}

// int main() {
// 	ZtGeographyCoordinateTransform transform;
//     double x, y;
//     transform.BL2XY(1, 2, x, y);
//     std::cout << x << "," << y << std::endl;
//     double lon,lat;
//     transform.XY2BL(4429715.908012726,2,lat,lon);
//     std::cout <<"lon:"<< lon <<"\t lat: "<< lat << std::endl;
// 	return 0;
// }
