#include "geometry.h"
#include<cmath>
#include <bits/stdc++.h>

using namespace std;

// 20220826 两点间距离公式
double getDistance(double x1, double y1, double x2, double y2)
{
    
    return sqrt((x1 - x2) *(x1 - x2) + (y1 - y2) * (y1 - y2));
    //return sqrt(pow(x1 - x2, 2) + pow(y1 - y2, 2));
}


// 计算点到线段的距离，如果这个公式是对的，相比原来的方法就是降维打击
double pointToLineDistance(const GaussRoadPoint &startPoint, const GaussRoadPoint &stopPoint, const GaussRoadPoint &checkPoint)
{

    double lengthStart2Stop = getDistance(startPoint.GaussX, startPoint.GaussY, stopPoint.GaussX, stopPoint.GaussY);
    double lengthStart2Check = getDistance(startPoint.GaussX, startPoint.GaussY, checkPoint.GaussX, checkPoint.GaussY);
    double lengthStop2Check = getDistance(stopPoint.GaussX, stopPoint.GaussY, checkPoint.GaussX, checkPoint.GaussY);

    // 垂足在线段外，距离点到线段端点的距离
    if (lengthStart2Check * lengthStart2Check > lengthStop2Check * lengthStop2Check + lengthStart2Stop * lengthStart2Stop)
        return lengthStop2Check;

    if (lengthStop2Check * lengthStop2Check > lengthStart2Check * lengthStart2Check + lengthStart2Stop * lengthStart2Stop)
        return lengthStart2Check;

    // 垂足在线段内  ，最短距离是三角形ABC以边BC的高，可通过海伦公式先求出面积，再求出高得到答案。
    double l = (lengthStart2Stop + lengthStart2Check + lengthStop2Check) / 2;
    double s = sqrt(l * (l - lengthStart2Stop) * (l - lengthStart2Check) * (l - lengthStop2Check));
    // std::cout<< "new pointToLineDistance++++++ "<< 2*s/lengthStart2Stop <<std:: endl;
    return 2 * s / lengthStart2Stop;
}

double pointToPolygon(const std::vector<GaussRoadPoint> pointPolygon, const GaussRoadPoint checkPoint)
{
    //只是进行数据类型的转换，调用网上成熟源码
    point_t po;
    po.x = checkPoint.GaussY;
    po.y = checkPoint.GaussX;

    if(pointPolygon.size() == 0)
        return 0;

    int n = pointPolygon.size();    

    point_t * p = new point_t[n+1];//最后一点是回到起点的点
    for(int i=0; i< n; i++)
    {
        p[i].x = pointPolygon[i].GaussY;
        p[i].y = pointPolygon[i].GaussX;
    }
    p[n] = p[0];

    //检查是否在多边形内部
    if ( isInSimple( p ,n , po ) )
    {
            delete [] p;
            return 0;
    }

    //计算点到凸多边形的距离
    double ans = PointTOline( p[0],p[1],po );
    
    for (int i = 1;i < n ;++i)
            ans = min( ans, PointTOline(p[i] , p[i+1] , po) );

    
    delete [] p;
    //cout <<"pointToPolygon ans =" <<ans<<endl;
    return ans;


 
}


////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

double cross(point_t const &O,point_t const &A,point_t const &B){
	double xOA = A.x - O.x;
	double yOA = A.y - O.y;
	double xOB = B.x - O.x;
	double yOB = B.y - O.y;
	return xOA * yOB - xOB * yOA;
}

double PointTOline( point_t const&a,point_t const&b,point_t const&p){
    double ap_ab = (b.x - a.x)*(p.x - a.x)+(b.y - a.y)*(p.y - a.y);//cross( a , p , b );
    if ( ap_ab <= 0 )
        return sqrt( (p.x-a.x)*(p.x-a.x) + (p.y-a.y)*(p.y-a.y) );
 
    double d2 = ( b.x - a.x ) * ( b.x - a.x ) + ( b.y-a.y ) * ( b.y-a.y ) ;
    if ( ap_ab >= d2 ) return sqrt( (p.x - b.x )*( p.x - b.x ) + ( p.y - b.y )*( p.y - b.y ) ) ;
 
    double r = ap_ab / d2;
    double px = a.x + ( b.x - a.x ) *r;
    double py = a.y + ( b.y - a.y ) *r;
    return sqrt( (p.x - px)*(p.x - px) + (p.y - py)*(p.y - py) );
}
 
bool isOnline( point_t const&a,point_t const&b, point_t const&po ){
    return po.x >= min( a.x , b.x ) &&
           po.x <= max( a.x , b.x ) &&
           po.y >= min( a.y , b.y ) &&
           po.y <= max( a.y , b.y ) &&
           ( po.x - a.x ) * ( b.y - a.y ) == ( po.y - a.y ) * ( b.x - a.x );
}

bool isInSimple( point_t * p ,int n , point_t const&po ){
    p[n] = p[0];
    bool flag = 0;
    int tmp;
    for ( int i = 0; i < n;++i ){
        if ( isOnline( p[i] , p[i+1] , po ) ) return true;

        if ( p[i].y == p[i+1].y ) continue;

        p[i].y < p[i+1].y ? tmp = i+1 : tmp = i ;
        if ( po.y == p[tmp].y && po.x < p[tmp].x ) flag ^= 1;
        p[i].y > p[i+1].y ? tmp = i+1 : tmp = i ;
        if ( po.y == p[tmp].y && po.x < p[tmp].x ) continue ;
 
        if ( po.x < max( p[i].x , p[i+1].x ) &&
             po.y > min( p[i].y , p[i+1].y ) &&
             po.y < max( p[i].y , p[i+1].y ) ) flag ^= 1;
    }
    return flag;
}
 
bool isLineIntersection(GaussRoadPoint line1StartPoint,  GaussRoadPoint line1EndPoint, GaussRoadPoint line2StartPoint,  GaussRoadPoint line2EndPoint)
{

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
        return false;
    }
    
    //跨立实验
    if ((((l1.x1 - l2.x1)*(l2.y2 - l2.y1) - (l1.y1 - l2.y1)*(l2.x2 - l2.x1))*
        ((l1.x2 - l2.x1)*(l2.y2 - l2.y1) - (l1.y2 - l2.y1)*(l2.x2 - l2.x1))) > 0 ||
        (((l2.x1 - l1.x1)*(l1.y2 - l1.y1) - (l2.y1 - l1.y1)*(l1.x2 - l1.x1))*
        ((l2.x2 - l1.x1)*(l1.y2 - l1.y1) - (l2.y2 - l1.y1)*(l1.x2 - l1.x1))) > 0)
    {
        return false;
    }
    return true;
}

bool isPolygonsIntersection(std::vector<GaussRoadPoint>  polygon1Point,  std::vector< GaussRoadPoint> polygon2Point)
{
    if(polygon1Point.empty() || polygon2Point.empty())
        return false;

    //把第一个点补充在队尾，用于循环线段回到起点
    polygon1Point.push_back(polygon1Point.front());
    polygon2Point.push_back(polygon2Point.front());

    for(int i=0; i<(int)polygon1Point.size()-1;i++)
    {
        for(int j=0; j<(int)polygon2Point.size()-1;j++)
        {
            if(isLineIntersection(polygon1Point[i], polygon1Point[i+1], polygon2Point[i],  polygon2Point[i+1]))
                return true;
        }
    }

    return false;
   
}

//在里面是true
bool isPointInPolygon(const std::vector<GaussRoadPoint> pointPolygon, const GaussRoadPoint checkPoint)
{
    int nvert = (int)pointPolygon.size();
    double * vertx = new double[nvert];
    double * verty = new double[nvert];

    for(int i=0;i< nvert; i++)
    {
        vertx[i] = pointPolygon[i].GaussY;
        verty[i] = pointPolygon[i].GaussX;
    }

    bool result =  InOrOutPolygon(nvert, vertx, verty, checkPoint.GaussY, checkPoint.GaussX) ;

    delete []  vertx;
    delete [] verty;

   return (!result);
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

bool  InOrOutPolygon(int nvert, double *vertx, double *verty, double testx, double testy)
{
    int i, j, crossings = 0;
    crossings = 0;
    double * x1 = new double[nvert];
    double * y1 = new double[nvert];
    for (i = 0, j = nvert-1; i < nvert; j = i++) 
    {
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
    return (crossings % 2 != 0);
}


