#include "geometry_m.h"
#include <cmath>
#include <bits/stdc++.h>

using namespace std;

namespace planning{

/**
 * @brief 几何关系函数
 * @file geometry_m.cpp
 * @version 0.0.1
 * @author Zehang Zhu (zzh22@mails.tsinghua.edu.cn)
 * @date 2023-11-25
 */

/**
 * @brief 计算点到凸多边形的距离
 * @param[IN] param 无
 * @param[IN] input 多边形顶点向量数组，检查点
 * @param[OUT] output 点到凸多边形的距离
 
 * @cn_name: 计算点到凸多边形的距离
 
 * @granularity: composite //函数组件的粒度， atomic： 基础组件（原子），composite>： 复合组件（在前端可以向下一层展开）
 
 * @tag:planning
 */
void pointToPolygon(const pointToPolygonParam &param, const pointToPolygonInput &input, pointToPolygonOutput &output){
    std::vector<GaussRoadPoint> pointPolygon = input.pointPolygon;
    GaussRoadPoint checkPoint = input.checkPoint;

    //只是进行数据类型的转换，调用网上成熟源码
    point_t po;
    po.x = checkPoint.GaussY;
    po.y = checkPoint.GaussX;

    if(pointPolygon.size() == 0){
        output.dis = 0;
        return;
    }

    int n = pointPolygon.size();    

    point_t * p = new point_t[n+1];//最后一点是回到起点的点
    for(int i=0; i< n; i++)
    {
        p[i].x = pointPolygon[i].GaussY;
        p[i].y = pointPolygon[i].GaussX;
    }
    p[n] = p[0];

    isInSimpleParam param0{
    };

    isInSimpleInput input0{
       p, n, po
    };

    isInSimpleOutput output0{
       false
    };

    isInSimple(param0, input0, output0);
    //检查是否在多边形内部
    if (output0.flag){
        delete [] p;
        output.dis = 0;
        return;
    }

    pointToLineParam param1{
    };

    pointToLineInput input1{
        p[0], p[1], po
    };

    pointToLineOutput output1{
        0
    };

    //计算点到凸多边形的距离
    pointToLine(param1, input1, output1);
    double ans = output1.value;
    //double ans = PointTOline(p[0], p[1], po);
    
    for (int i = 1;i < n ;++i){
        pointToLineParam param1{
        };

        pointToLineInput input1{
            p[i], p[i+1], po
        };

        pointToLineOutput output1{
            0
        };

        //计算点到凸多边形的距离
        pointToLine(param1, input1, output1);

        ans = min(ans, output1.value);
    }

    
    delete [] p;
    //cout <<"pointToPolygon ans =" <<ans<<endl;
    output.dis = ans;
    return;
}


////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/**
 * @brief 点乘
 * @param[IN] param 无
 * @param[IN] input 向量原点，
 * @param[OUT] output 点到凸多边形的距离
 
 * @cn_name: 点乘
 
 * @granularity: atomic //函数组件的粒度， atomic： 基础组件（原子），composite>： 复合组件（在前端可以向下一层展开）
 
 * @tag:planning
 */
void cross(const crossParam &param, const crossInput &input, crossOutput &output){
    point_t O = input.O;
    point_t A = input.A;
    point_t B = input.B;

	double xOA = A.x - O.x;
	double yOA = A.y - O.y;
	double xOB = B.x - O.x;
	double yOB = B.y - O.y;
    output.value = xOA * yOB - xOB * yOA;
	return;
}

/**
 * @brief 计算点到线段的距离
 * @param[IN] param 无
 * @param[IN] input 线段起点，线段终点，检查点
 * @param[OUT] output 点到线段的距离
 
 * @cn_name: 计算点到线段的距离
 
 * @granularity: atomic //函数组件的粒度， atomic： 基础组件（原子），composite>： 复合组件（在前端可以向下一层展开）
 
 * @tag:planning
 */
void pointToLine(const pointToLineParam &param, const pointToLineInput &input, pointToLineOutput &output){
    point_t a = input.a;
    point_t b = input.b;
    point_t p = input.p;

    double ap_ab = (b.x - a.x)*(p.x - a.x)+(b.y - a.y)*(p.y - a.y);//cross( a , p , b );
    if ( ap_ab <= 0 ){
        output.value = sqrt( (p.x-a.x)*(p.x-a.x) + (p.y-a.y)*(p.y-a.y) );
        return;
    }
 
    double d2 = ( b.x - a.x ) * ( b.x - a.x ) + ( b.y-a.y ) * ( b.y-a.y ) ;
    if ( ap_ab >= d2 ){
        output.value = sqrt( (p.x - b.x )*( p.x - b.x ) + ( p.y - b.y )*( p.y - b.y ) ) ;
        return;
    }
 
    double r = ap_ab / d2;
    double px = a.x + ( b.x - a.x ) *r;
    double py = a.y + ( b.y - a.y ) *r;
    output.value = sqrt( (p.x - px)*(p.x - px) + (p.y - py)*(p.y - py) );
    return;
}

/**
 * @brief 判断点是否在线段上
 * @param[IN] param 无
 * @param[IN] input 线段起点，线段终点，检查点
 * @param[OUT] output 判断结果
 
 * @cn_name: 判断点是否在线段上
 
 * @granularity: atomic //函数组件的粒度， atomic： 基础组件（原子），composite>： 复合组件（在前端可以向下一层展开）
 
 * @tag:planning
 */ 
void isOnline(const isOnlineParam &param, const isOnlineInput &input, isOnlineOutput &output){
    point_t a = input.a;
    point_t b = input.b;
    point_t po = input.po;

    output.flag = po.x >= min( a.x , b.x ) &&
                   po.x <= max( a.x , b.x ) &&
                   po.y >= min( a.y , b.y ) &&
                   po.y <= max( a.y , b.y ) &&
                   ( po.x - a.x ) * ( b.y - a.y ) == ( po.y - a.y ) * ( b.x - a.x );
    return;
}

/**
 * @brief 判断点是否在横平竖直的多边形内部
 * @param[IN] param 无
 * @param[IN] input 多边形顶点，多边形顶点数，检查点
 * @param[OUT] output 判断结果
 
 * @cn_name: 判断点是否在横平竖直的多边形内部
 
 * @granularity: composite //函数组件的粒度， atomic： 基础组件（原子），composite>： 复合组件（在前端可以向下一层展开）
 
 * @tag:planning
 */ 
void isInSimple(const isInSimpleParam &param, const isInSimpleInput &input, isInSimpleOutput &output){
    point_t *p = input.p;
    int n = input.n;
    point_t po = input.po;

    p[n] = p[0];
    bool flag = 0;
    int tmp;
    for ( int i = 0; i < n;++i ){

        isOnlineParam param0{
        };

        isOnlineInput input0{
            p[i] , p[i+1] , po
        };

        isOnlineOutput output0{
            false
        };

        isOnline(param0, input0, output0);

        if ( output0.flag ){
            output.flag = true;
            return;
        }

        if ( p[i].y == p[i+1].y ) continue;

        p[i].y < p[i+1].y ? tmp = i+1 : tmp = i ;
        if ( po.y == p[tmp].y && po.x < p[tmp].x ) flag ^= 1;
        p[i].y > p[i+1].y ? tmp = i+1 : tmp = i ;
        if ( po.y == p[tmp].y && po.x < p[tmp].x ) continue ;
 
        if ( po.x < max( p[i].x , p[i+1].x ) &&
             po.y > min( p[i].y , p[i+1].y ) &&
             po.y < max( p[i].y , p[i+1].y ) ) flag ^= 1;
    }
    output.flag = flag;
    return;
}

/**
 * @brief 判断两线段是否相交
 * @param[IN] param 无
 * @param[IN] input 两线段起点、终点
 * @param[OUT] output 判断结果
 
 * @cn_name: 判断两线段是否相交
 
 * @granularity: atomic //函数组件的粒度， atomic： 基础组件（原子），composite>： 复合组件（在前端可以向下一层展开）
 
 * @tag:planning
 */ 
void isLineIntersection(const isLineIntersectionParam &param, const isLineIntersectionInput &input, isLineIntersectionOutput &output){
    GaussRoadPoint line1StartPoint = input.line1StartPoint;
    GaussRoadPoint line1EndPoint = input.line1EndPoint;
    GaussRoadPoint line2StartPoint = input.line2StartPoint;
    GaussRoadPoint line2EndPoint = input.line2EndPoint;

    struct Line {
        double x1;
        double y1;
        double x2;
        double y2;
    } l1,l2;

    l1.x1 = line1StartPoint.GaussY;
    l1.y1 = line1StartPoint.GaussX;
    l1.x2 = line1EndPoint.GaussY;
    l1.y2 = line1EndPoint.GaussX;

    l2.x1 = line2StartPoint.GaussY;
    l2.y1 = line2StartPoint.GaussX;
    l2.x2 = line2EndPoint.GaussY;
    l2.y2 = line2EndPoint.GaussX;


    //快速排斥实验
    if ((l1.x1 > l1.x2 ? l1.x1 : l1.x2) < (l2.x1 < l2.x2 ? l2.x1 : l2.x2) ||
        (l1.y1 > l1.y2 ? l1.y1 : l1.y2) < (l2.y1 < l2.y2 ? l2.y1 : l2.y2) ||
        (l2.x1 > l2.x2 ? l2.x1 : l2.x2) < (l1.x1 < l1.x2 ? l1.x1 : l1.x2) ||
        (l2.y1 > l2.y2 ? l2.y1 : l2.y2) < (l1.y1 < l1.y2 ? l1.y1 : l1.y2))
    {
        output.flag = false;
        return;
    }
    
    //跨立实验
    if ((((l1.x1 - l2.x1)*(l2.y2 - l2.y1) - (l1.y1 - l2.y1)*(l2.x2 - l2.x1))*
        ((l1.x2 - l2.x1)*(l2.y2 - l2.y1) - (l1.y2 - l2.y1)*(l2.x2 - l2.x1))) > 0 ||
        (((l2.x1 - l1.x1)*(l1.y2 - l1.y1) - (l2.y1 - l1.y1)*(l1.x2 - l1.x1))*
        ((l2.x2 - l1.x1)*(l1.y2 - l1.y1) - (l2.y2 - l1.y1)*(l1.x2 - l1.x1))) > 0)
    {
        output.flag = false;
        return;
    }
    output.flag = true;
    return;
}

/**
 * @brief 判断两多边形是否相交
 * @param[IN] param 无
 * @param[IN] input 两多边形顶点
 * @param[OUT] output 判断结果
 
 * @cn_name: 判断两多边形是否相交
 
 * @granularity: composite //函数组件的粒度， atomic： 基础组件（原子），composite>： 复合组件（在前端可以向下一层展开）
 
 * @tag:planning
 */ 
void isPolygonsIntersection(const isPolygonsIntersectionParam &param, const isPolygonsIntersectionInput &input, isPolygonsIntersectionOutput &output){
    std::vector<GaussRoadPoint> polygon1Point = input.polygon1Point;
    std::vector<GaussRoadPoint> polygon2Point = input.polygon2Point;
    if(polygon1Point.empty() || polygon2Point.empty()){
        output.flag = false;
        return;
    }

    //把第一个点补充在队尾，用于循环线段回到起点
    polygon1Point.push_back(polygon1Point.front());
    polygon2Point.push_back(polygon2Point.front());

    for(int i=0; i<(int)polygon1Point.size()-1;i++){
        for(int j=0; j<(int)polygon2Point.size()-1;j++){

            isLineIntersectionParam param0{
            };

            isLineIntersectionInput input0{
                polygon1Point[i], 
                polygon1Point[i+1], 
                polygon2Point[i],  
                polygon2Point[i+1]
            };

            isLineIntersectionOutput output0{
               false
            };    

            isLineIntersection(param0, input0, output0);
                    
            if(output0.flag){
                output.flag = true;
                return;
            }
        }
    }

    output.flag = false;
    return;
   
}


//在里面是true
/**
 * @brief 判断点是否在多边形内部
 * @param[IN] param 无
 * @param[IN] input 多边形顶点、检查点
 * @param[OUT] output 判断结果
 
 * @cn_name: 判断点是否在多边形内部
 
 * @granularity: composite //函数组件的粒度， atomic： 基础组件（原子），composite>： 复合组件（在前端可以向下一层展开）
 
 * @tag:planning
 */ 
void isPointInPolygon(const isPointInPolygonParam &param, const isPointInPolygonInput &input, isPointInPolygonOutput &output){
    std::vector<GaussRoadPoint> pointPolygon = input.pointPolygon;
    GaussRoadPoint checkPoint = input.checkPoint;  

    int nvert = (int)pointPolygon.size();
    double * vertx = new double[nvert];
    double * verty = new double[nvert];

    for(int i=0;i< nvert; i++){
        vertx[i] = pointPolygon[i].GaussY;
        verty[i] = pointPolygon[i].GaussX;
    }

    inOrOutPolygonParam param0{
    };

    inOrOutPolygonInput input0{
        nvert, vertx, verty, checkPoint.GaussY, checkPoint.GaussX
    };

    inOrOutPolygonOutput output0{
        false
    };

    inOrOutPolygon(param0, input0, output0);
    bool result = output0.flag;

    delete [] vertx;
    delete [] verty;

    output.flag = !result;
    return;
}

/************************************************************
** 函数名称:  InOrOutPolygon
** 功能描述:  判断点在多边形内外
** 输入参数:  nvert 顶点个数 vertx 多边形顶点x坐标数组 verty 多边形顶点y坐标数组
              testx 被判断点位置x坐标 testy 被判断点位置y坐标
** 输出参数:  NULL
** 返 回 值:  0:外 1:内
** 作    者:
** 日    期:  2018年3月21日
**************************************************************/
/**
 * @brief 判断点是否在多边形内部
 * @param[IN] param 无
 * @param[IN] input 多边形顶点数、多边形顶点X值，多边形顶点Y值，检查点X值，检查点Y值
 * @param[OUT] output 判断结果
 
 * @cn_name: 判断点是否在多边形内部
 
 * @granularity: atomic //函数组件的粒度， atomic： 基础组件（原子），composite>： 复合组件（在前端可以向下一层展开）
 
 * @tag:planning
 */ 
void inOrOutPolygon(const inOrOutPolygonParam &param, const inOrOutPolygonInput &input, inOrOutPolygonOutput &output){
    int nvert = input.nvert;
    double *vertx = input.vertx;
    double *verty = input.verty; 
    double testx = input.testx;
    double testy = input.testy;

    int i, j, crossings = 0;
    crossings = 0;
    double * x1 = new double[nvert];
    double * y1 = new double[nvert];
    for (i = 0, j = nvert-1; i < nvert; j = i++) {
    // 点在两个x之间 且以点垂直y轴向上做射线
    x1[i] = vertx[i];
    x1[j] = vertx[j];
    y1[i] = verty[i];
    y1[j] = verty[j];
    if((((vertx[i] < testx) && (vertx[j] >= testx))||((vertx[i] >= testx) && (vertx[j] < testx)))     
    &&   (testx > (vertx[j]-vertx[i]) * (testy-verty[i]) / (verty[j]-verty[i]) + vertx[i]))
       crossings++;
    }

    delete [] x1;
    delete [] y1;

    std::cout << "crossings" << crossings <<std::endl;
    output.flag = (crossings % 2 != 0);
    return;
}


}