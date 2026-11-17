#ifndef GEO_TRN_DIRECT_DISCTET_TYPES_H
#define GEO_TRN_DIRECT_DISCTET_TYPES_H

#include "baseTypes.h"

/** @brief 路径点最大数量 */
#ifndef PATH_PT_MAX
#define PATH_PT_MAX 100
#endif

/** @brief 除零保护精度 */
#ifndef GEO_EPSILON
#define GEO_EPSILON 1e-6f
#endif
/** @brief 圆周率 */
#ifndef APA_PI
#define APA_PI 3.14159265f
#endif

/**
 * @brief 弧线离散化输入
 * @en_name GeoTrnDirectDisctetInput
 * @cn_name 弧线离散化输入
 * @tag STRUCT
 * @field name=Crv, type=Real, unit=1/m, desc=曲率
 * @field name=CcX, type=Real, unit=m, desc=圆心X
 * @field name=CcY, type=Real, unit=m, desc=圆心Y
 * @field name=ArcLen, type=Real, unit=m, desc=弧长
 * @field name=GoDir, type=Integer, unit=1, desc=行驶方向1前进负1后退
 * @field name=StepLen, type=Real, unit=m, desc=离散步长
 * @field name=ArcAngle, type=Real, unit=rad, desc=圆弧角度
 * @field name=StPtX, type=Real, unit=m, desc=弧线起点X
 * @field name=StPtY, type=Real, unit=m, desc=弧线起点Y
 * @field name=StPtTheta, type=Real, unit=rad, desc=弧线起点航向
 * @field name=EdPtX, type=Real, unit=m, desc=弧线终点X
 * @field name=EdPtY, type=Real, unit=m, desc=弧线终点Y
 * @field name=EdPtTheta, type=Real, unit=rad, desc=弧线终点航向
 * @field name=LstPtX, type=Real, unit=m, desc=上一离散点X
 * @field name=LstPtY, type=Real, unit=m, desc=上一离散点Y
 * @field name=LstPtTheta, type=Real, unit=rad, desc=上一离散点航向
 * @field name=VehPosX, type=Real, unit=m, desc=车辆位姿X
 * @field name=VehPosY, type=Real, unit=m, desc=车辆位姿Y
 * @field name=VehPosTheta, type=Real, unit=rad, desc=车辆位姿航向
 * @field name=SltTyp, type=Integer, unit=1, desc=车位类型0垂直1平行
 * @field name=LstStp, type=Integer, unit=1, desc=末段标志1最后一段
 * @field name=TarPtX, type=Real, unit=m, desc=目标点X
 * @field name=TarPtY, type=Real, unit=m, desc=目标点Y
 * @field name=TarPtTheta, type=Real, unit=rad, desc=目标点航向
 * @field name=InPtCount, type=Integer, unit=1, desc=进入时已有点数
 * @field name=InPtX, type=Array<Real, [100]>, unit=m, desc=进入时点X数组
 * @field name=InPtY, type=Array<Real, [100]>, unit=m, desc=进入时点Y数组
 * @field name=InPtTht, type=Array<Real, [100]>, unit=rad, desc=进入时点航向数组
 * @field name=InPtOdo, type=Array<Real, [100]>, unit=m, desc=进入时点里程数组
 * @version 2.0
 * @date 2026-05-01
 * @author aide
 */
typedef struct {
  Real Crv;
  Real CcX;
  Real CcY;
  Real ArcLen;
  Integer GoDir;
  Real StepLen;
  Real ArcAngle;
  Real StPtX;
  Real StPtY;
  Real StPtTheta;
  Real EdPtX;
  Real EdPtY;
  Real EdPtTheta;
  Real LstPtX;
  Real LstPtY;
  Real LstPtTheta;
  Real VehPosX;
  Real VehPosY;
  Real VehPosTheta;
  Integer SltTyp;
  Integer LstStp;
  Real TarPtX;
  Real TarPtY;
  Real TarPtTheta;
  Integer InPtCount;
  Real InPtX[PATH_PT_MAX];
  Real InPtY[PATH_PT_MAX];
  Real InPtTht[PATH_PT_MAX];
  Real InPtOdo[PATH_PT_MAX];
} GeoTrnDirectDisctetInput_t;

/**
 * @brief 弧线离散化输出
 * @en_name GeoTrnDirectDisctetOutput
 * @cn_name 弧线离散化输出
 * @tag STRUCT
 * @field name=PtCount, type=Integer, unit=1, desc=有效路径点数量
 * @field name=PthPtX, type=Array<Real, [100]>, unit=m, desc=离散后点X数组
 * @field name=PthPtY, type=Array<Real, [100]>, unit=m, desc=离散后点Y数组
 * @field name=PthPtTht, type=Array<Real, [100]>, unit=rad, desc=离散后点航向数组
 * @field name=PthPtOdo, type=Array<Real, [100]>, unit=m, desc=离散后点里程数组
 * @field name=LstPtX, type=Real, unit=m, desc=最后离散点X
 * @field name=LstPtY, type=Real, unit=m, desc=最后离散点Y
 * @field name=LstPtTheta, type=Real, unit=rad, desc=最后离散点航向
 * @version 2.0
 * @date 2026-05-01
 * @author aide
 */
typedef struct {
  Integer PtCount;
  Real PthPtX[PATH_PT_MAX];
  Real PthPtY[PATH_PT_MAX];
  Real PthPtTht[PATH_PT_MAX];
  Real PthPtOdo[PATH_PT_MAX];
  Real LstPtX;
  Real LstPtY;
  Real LstPtTheta;
} GeoTrnDirectDisctetOutput_t;

void geoTrnDirectDisctet(const GeoTrnDirectDisctetInput_t *input,
                         GeoTrnDirectDisctetOutput_t *output);

#endif
