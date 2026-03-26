#include "ztgeographycoordinatetransform_m.h"

/*
*    此处未定义PI，直接使用PI值，防止与其他文件宏定义冲突
*/

// ZtGeographyCoordinateTransform::ZtGeographyCoordinateTransform()
//     : meridianLine(-360), projType('g')
// {

// }

// ZtGeographyCoordinateTransform::~ZtGeographyCoordinateTransform()
// {

// }

void XY2BL(const ZtGeographyCoordinateTransform::XY2BLParam &param, 
            const ZtGeographyCoordinateTransform::XY2BLInput &input, 
            ZtGeographyCoordinateTransform::XY2BLOutput &output)
{
    EllipsoidParameter ellipPmt = param.ellipPmt;

    double meridianLine = param.meridianLine;
    char projType = param.projType; 
    double x = input.x;
    double y = input.y;
    if (projType == 'u')
    {
        y = y / 0.9996;
    }

    double bf0 = y / ellipPmt.a0, bf;
    double threshould = 1.0;
    while (threshould > 0.00000001)
    {
        double y0 = -ellipPmt.a2 * sin(2 * bf0) / 2 + ellipPmt.a4 * sin(4 * bf0) / 4 - ellipPmt.a6 * sin(6 * bf0) / 6;
        bf = (y - y0) / ellipPmt.a0;
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

    x = x - 500000;
    if (projType == 'u')
    {
        x = x / 0.9996;
    }

    double temp0, temp1, temp2;
    temp0 = t * x * x / (2 * m * n);
    temp1 = t * (5 + 3 * t * t + j2 - 9 * j2 * t * t) * pow(x, 4) / (24 * m * pow(n, 3));
    temp2 = t * (61 + 90 * t * t + 45 * pow(t, 4)) * pow(x, 6) / (720 * pow(n, 5) * m);
    output.lat = (bf - temp0 + temp1 - temp2) * 57.29577951308232;

    temp0 = x / (n*cos(bf));
    temp1 = (1 + 2 * t * t + j2) * pow(x, 3) / (6 * pow(n, 3) * cos(bf));
    temp2 = (5 + 28 * t * t + 6 * j2 + 24 * pow(t, 4) + 8 * t * t * j2) * pow(x, 5) / (120 * pow(n, 5) * cos(bf));
    output.lon = (temp0 - temp1 + temp2) * 57.29577951308232 + meridianLine;
    output.isValidCoord = true;
    return;
}

void BL2XY(const ZtGeographyCoordinateTransform::BL2XYParam &param, 
            const ZtGeographyCoordinateTransform::BL2XYInput &input, 
            ZtGeographyCoordinateTransform::BL2XYOutput &output)
{
    double meridianLine = param.meridianLine;
    EllipsoidParameter ellipPmt = param.ellipPmt;
    char projType = param.projType; 
    double lon = input.lon;
    double lat = input.lat;
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

    double temp[6] = { 0 };
    temp[0] = n * sin(lat) * cos(lat) * dL * dL / 2;
    temp[1] = n * sin(lat) * pow(cos(lat), 3) * (5 - tn2 + 9 * j2 + 4 * j2 * j2) * pow(dL, 4) / 24;
    temp[2] = n * sin(lat) * pow(cos(lat), 5) * (61 - 58 * tn2 + tn4) * pow(dL, 6) / 720;
    temp[3] = n * cos(lat) * dL;
    temp[4] = n * pow(cos(lat), 3) * (1 - tn2 + j2) * pow(dL, 3) / 6;
    temp[5] = n * pow(cos(lat), 5) * (5 - 18 * tn2 + tn4 + 14 * j2 - 58 * tn2 * j2) * pow(dL, 5) / 120;

    double y = X + temp[0] + temp[1] + temp[2];
    double x = temp[3] + temp[4] + temp[5];

    if (projType == 'g')
    {
        x = x + 500000;
    }
    else if (projType == 'u')
    {
        x = x * 0.9996 + 500000;
        y = y * 0.9996;
    }

    output.x = x;
    output.y = y;
    output.isValidCoord = true;

    return;
}

void XYZ2BLH(const ZtGeographyCoordinateTransform::XYZ2BLHParam &param, 
              const ZtGeographyCoordinateTransform::XYZ2BLHInput &input, 
              ZtGeographyCoordinateTransform::XYZ2BLHOutput &output)
{
    EllipsoidParameter ellipPmt = param.ellipPmt;
    double x = input.x;
    double y = input.y;
    double z = input.z;
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
    output.ht = sqrt(x * x + y * y) / cos(nowB) - nowN;
    output.lon = atan2(y, x) * 57.29577951308232;    // 180 / pi
    output.lat = nowB * 57.29577951308232;
    output.isValidCoord = true;
    return;
}

void BLH2XYZ(const ZtGeographyCoordinateTransform::BLH2XYZParam &param, 
              const ZtGeographyCoordinateTransform::BLH2XYZInput &input, 
              ZtGeographyCoordinateTransform::BLH2XYZOutput &output)
{
    EllipsoidParameter ellipPmt = param.ellipPmt;
    double lat = input.lat;
    double lon = input.lon;
    double ht = input.ht;
    double sinB = sin(lat / 57.29577951308232);
    double cosB = cos(lat / 57.29577951308232);
    double sinL = sin(lon / 57.29577951308232);
    double cosL = cos(lon / 57.29577951308232);

    double N = ellipPmt.a / sqrt(1.0 - ellipPmt.e2 * sinB * sinB);
    output.x = (N + ht) * cosB * cosL;
    output.y = (N + ht) * cosB * sinL;
    output.z = (N * ellipPmt.b * ellipPmt.b / (ellipPmt.a * ellipPmt.a) + ht) * sinB;
    output.isValidCoord = true;

    return;
}
