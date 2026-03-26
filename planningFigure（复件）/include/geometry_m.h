#pragma once
#ifndef GEOMETRY_M_H
#define GEOMETRY_M_H

#include"localization_MapAnalysis.h"
#include <cmath>
#include <bits/stdc++.h>
// 20220826
// 计算欧式距离

/**
 * \namespace geometry
 * \brief 这是一个计算欧式距离的namespace。
 */
// namespace geometry{

   struct pointToPolygonParam{
   };

   struct pointToPolygonInput{
      std::vector<GaussRoadPoint> pointPolygon;
      GaussRoadPoint checkPoint;
   };

   struct pointToPolygonOutput{
      double dis;
   };
   //计算点到多边形的距离
   void pointToPolygon(const pointToPolygonParam &param, const pointToPolygonInput &input, pointToPolygonOutput &output);

   struct isLineIntersectionParam{
   };

   struct isLineIntersectionInput{
      GaussRoadPoint line1StartPoint;
      GaussRoadPoint line1EndPoint;
      GaussRoadPoint line2StartPoint;
      GaussRoadPoint line2EndPoint;
   };

   struct isLineIntersectionOutput{
      bool flag;
   };
   //判断线段是否相交
   void isLineIntersection(const isLineIntersectionParam &param, const isLineIntersectionInput &input, isLineIntersectionOutput &output);

   struct isPolygonsIntersectionParam{
   };

   struct isPolygonsIntersectionInput{
      std::vector<GaussRoadPoint> polygon1Point;
      std::vector<GaussRoadPoint> polygon2Point;
   };

   struct isPolygonsIntersectionOutput{
      bool flag;
   };
   //判断多边形是否相交，简单地进行每个线段相交的检验，不考虑两个多边形完全相同和包含的关系
   void isPolygonsIntersection(const isPolygonsIntersectionParam &param, const isPolygonsIntersectionInput &input, isPolygonsIntersectionOutput &output);

   struct isPointInPolygonParam{
   };

   struct isPointInPolygonInput{
      std::vector<GaussRoadPoint> pointPolygon;
      GaussRoadPoint checkPoint;
   };

   struct isPointInPolygonOutput{
      bool flag;
   };
   //判断点是否在多边形内,返回值true 在内部
   void isPointInPolygon(const isPointInPolygonParam &param, const isPointInPolygonInput &input, isPointInPolygonOutput &output);

   ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
   //网上原始函数 
   //https://blog.csdn.net/betwater/article/details/52434017

   struct point_t {
      double x,y;
   };

   struct crossParam{
   };

   struct crossInput{
      point_t O;
      point_t A;
      point_t B;
   };

   struct crossOutput{
      double value;
   };

   void cross(const crossParam &param, const crossInput &input, crossOutput &output);

   struct pointToLineParam{
   };

   struct pointToLineInput{
      point_t a;
      point_t b;
      point_t p;
   };

   struct pointToLineOutput{
      double value;
   };

   void pointToLine(const pointToLineParam &param, const pointToLineInput &input, pointToLineOutput &output);

   struct isOnlineParam{
   };

   struct isOnlineInput{
      point_t a;
      point_t b;
      point_t po;
   };

   struct isOnlineOutput{
      bool flag;
   };

   void isOnline(const isOnlineParam &param, const isOnlineInput &input, isOnlineOutput &output);

   struct isInSimpleParam{
   };

   struct isInSimpleInput{
      point_t *p;
      int n;
      point_t po;
   };

   struct isInSimpleOutput{
      bool flag;
   };

   void isInSimple(const isInSimpleParam &param, const isInSimpleInput &input, isInSimpleOutput &output);
   
   struct inOrOutPolygonParam{
   };

   struct inOrOutPolygonInput{
      int nvert;
      double *vertx;
      double *verty;
      double testx;
      double testy;
   };

   struct inOrOutPolygonOutput{
      bool flag;
   };
   //https://www.cnblogs.com/-Donny/p/8651050.html
   void inOrOutPolygon(const inOrOutPolygonParam &param, const inOrOutPolygonInput &input, inOrOutPolygonOutput &output);
// }
#endif