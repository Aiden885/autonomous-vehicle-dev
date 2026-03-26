#pragma once
/*******************************************************************
*
*    作者:    Sun Zhenxing
*    创建日期:    20190819
*
*    说明：实现经纬度与平面坐标互转，实现经纬度与空间直角坐标互转
*
******************************************************************/

#ifndef ZTGEOGRAPHYCOORDINATETRANSFORM_H
#define ZTGEOGRAPHYCOORDINATETRANSFORM_H

#include <math.h>

struct EllipsoidParameter
{
    double a, b, f;
    double e2, ep2;

    // 高斯投影参数
    double c;
    double a0, a2, a4, a6;

    EllipsoidParameter()
    {
        // Default: wgs84
        a = 6378137.0;
        e2 = 0.00669437999013;

        b = sqrt(a * a * (1 - e2));
        ep2 = (a * a - b * b) / (b * b);
        f = (a - b) / a;
        double f0 = 1 / 298.257223563;
        double f1 = 1 / f;

        c = a / (1 - f);
        double m0, m2, m4, m6, m8;
        m0 = a * (1 - e2);
        m2 = 1.5 * e2 * m0;
        m4 = 1.25 * e2 * m2;
        m6 = 7 * e2 * m4 / 6;
        m8 = 9 * e2 * m6 / 8;
        a0 = m0 + m2 / 2 + 3 * m4 / 8 + 5 * m6 / 16 + 35 * m8 / 128;
        a2 = m2 / 2 + m4 / 2 + 15 * m6 / 32 + 7 * m8 / 16;
        a4 = m4 / 8 + 3 * m6 / 16 + 7 * m8 / 32;
        a6 = m6 / 32 + m8 / 16;
    }

    EllipsoidParameter(double ia, double ib)
    {
        if (ib > 1000000)    // ib 是短半轴
        {
            a = ia;
            b = ib;

            f = (a - b) / a;
            e2 = (a * a - b * b) / (a * a);
            ep2 = (a * a - b * b) / (b * b);
        }
        else if (ib < 1)    // ib 是椭球第一偏心率的平方
        {
            a = ia;
            e2 = ib;

            b = sqrt(a * a * (1 - e2));
            ep2 = (a * a - b * b) / (b * b);
            f = (a - b) / a;
        }

        c = a / (1 - f);
        double m0, m2, m4, m6, m8;
        m0 = a * (1 - e2);
        m2 = 1.5 * e2 * m0;
        m4 = 1.25 * e2 * m2;
        m6 = 7 * e2 * m4 / 6;
        m8 = 9 * e2 * m6 / 8;
        a0 = m0 + m2 / 2 + 3 * m4 / 8 + 5 * m6 / 16 + 35 * m8 / 128;
        a2 = m2 / 2 + m4 / 2 + 15 * m6 / 32 + 7 * m8 / 16;
        a4 = m4 / 8 + 3 * m6 / 16 + 7 * m8 / 32;
        a6 = m6 / 32 + m8 / 16;
    }
};

class ZtGeographyCoordinateTransform
{
public:
    ZtGeographyCoordinateTransform();
    ~ZtGeographyCoordinateTransform();

    EllipsoidParameter ellipPmt;

    double meridianLine;
    char projType;            // 'u': utm, 'g': gauss-kruger

                              /*
                              *    In projection coordinate system: x: east  y: north  z: height
                              */

    bool XY2BL(double x, double y, double &lat, double &lon);
    bool BL2XY(double lat, double lon, double &x, double &y);
    bool XYZ2BLH(double x, double y, double z, double &lat, double &lon, double &ht);
    bool BLH2XYZ(double lat, double lon, double ht, double &x, double &y, double &z);
};


#endif
