#pragma once

#include"localization_MapAnalysis.h"

// 20220826
// 计算欧式距离
double getDistance(double x1, double y1, double x2, double y2);
// 计算点到线段距离
double pointToLineDistance(const GaussRoadPoint &startPoint, const GaussRoadPoint &stopPoint, const GaussRoadPoint &checkPoint);

//计算点到多边形的距离
double pointToPolygon(const std::vector<GaussRoadPoint> pointPolygon, const GaussRoadPoint checkPoint);

//判断线段是否相交
bool isLineIntersection(GaussRoadPoint line1StartPoint,  GaussRoadPoint line1EndPoint, GaussRoadPoint line2StartPoint,  GaussRoadPoint line2EndPoint);

//判断多边形是否相交，简单地进行每个线段相交的检验，不考虑两个多边形完全相同和包含的关系
bool isPolygonsIntersection(std::vector<GaussRoadPoint>  polygon1Point,  std::vector< GaussRoadPoint> polygon2Point);

//判断点是否在多边形内,返回值true 在内部
bool isPointInPolygon(const std::vector<GaussRoadPoint> pointPolygon, const GaussRoadPoint checkPoint);
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//网上原始函数 
//https://blog.csdn.net/betwater/article/details/52434017
 struct point_t {
    double x,y;
};

double cross(point_t const &O,point_t const &A,point_t const &B);
double PointTOline( point_t const&a,point_t const&b,point_t const&p);
bool isOnline( point_t const&a,point_t const&b, point_t const&po );
bool isInSimple( point_t * p ,int n , point_t const&po );
 
 //https://www.cnblogs.com/-Donny/p/8651050.html
 bool InOrOutPolygon(int nvert, double *vertx, double *verty, double testx, double testy);
