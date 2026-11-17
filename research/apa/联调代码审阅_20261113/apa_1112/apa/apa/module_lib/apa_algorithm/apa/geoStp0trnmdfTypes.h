#ifndef GEO_STP0TRNMDF_TYPES_H
#define GEO_STP0TRNMDF_TYPES_H

#include "baseTypes.h"
#include "geoCircleIntrvCheckTypes.h"
#include "geoRev2CircleLinkTypes.h"
#include "geoStPtSpcVtTypes.h"

/** @brief 轨迹段最大数量 */
#ifndef APA_PATH_SEG_MAX
#define APA_PATH_SEG_MAX 30
#endif

/** @brief 圆心数组最大数量（每段2个） */
#ifndef APA_PATH_CC_MAX
#define APA_PATH_CC_MAX 60
#endif

/**
 * @brief stp0转向修正输入
 * @en_name GeoStp0trnmdfInput
 * @cn_name stp0转向修正输入
 * @tag STRUCT
 * @field name=VehX, type=Real, unit=m, desc=车辆X
 * @field name=VehY, type=Real, unit=m, desc=车辆Y
 * @field name=VehTheta, type=Real, unit=rad, desc=车辆航向
 * @field name=TarX, type=Real, unit=m, desc=目标X
 * @field name=TarY, type=Real, unit=m, desc=目标Y
 * @field name=TarTheta, type=Real, unit=rad, desc=目标航向
 * @field name=SltPt1X, type=Real, unit=m, desc=车位角点1X
 * @field name=SltPt1Y, type=Real, unit=m, desc=车位角点1Y
 * @field name=SltPt2X, type=Real, unit=m, desc=车位角点2X
 * @field name=SltPt2Y, type=Real, unit=m, desc=车位角点2Y
 * @field name=SltPt3X, type=Real, unit=m, desc=车位角点3X
 * @field name=SltPt3Y, type=Real, unit=m, desc=车位角点3Y
 * @field name=SltPt4X, type=Real, unit=m, desc=车位角点4X
 * @field name=SltPt4Y, type=Real, unit=m, desc=车位角点4Y
 * @field name=SltEntr, type=Integer, unit=1, desc=车位入口边编号
 * @field name=SltState, type=Integer, unit=1, desc=车位状态
 * @field name=RminDefault, type=Real, unit=m, desc=默认最小半径
 * @field name=RminSuccSlot, type=Real, unit=m, desc=成功车位最小半径
 * @field name=VehW, type=Real, unit=m, desc=车宽
 * @field name=VehLlF, type=Real, unit=m, desc=前悬到参考点长度
 * @field name=VehLfMin, type=Real, unit=m, desc=最小前向安全长度
 * @field name=PathSegNum, type=Integer, unit=1, desc=当前路径段数
 * @field name=PathGoDir, type=Array<Integer, [30]>, unit=1, desc=行驶方向数组
 * @field name=PathTrnDir, type=Array<Integer, [30]>, unit=1, desc=转向数组
 * @field name=PathCrv, type=Array<Real, [30]>, unit=1/m, desc=曲率数组
 * @field name=PathTht, type=Array<Real, [30]>, unit=rad, desc=转角数组
 * @field name=PathLen, type=Array<Real, [30]>, unit=m, desc=长度数组
 * @field name=PathStx, type=Array<Real, [30]>, unit=m, desc=起点X数组
 * @field name=PathSty, type=Array<Real, [30]>, unit=m, desc=起点Y数组
 * @field name=PathStTh, type=Array<Real, [30]>, unit=rad, desc=起点航向数组
 * @field name=PathCC, type=Array<Real, [60]>, unit=m, desc=圆心数组
 * @version 1.0
 * @date 2026-03-05
 * @author aide
 */
typedef struct {
  Real VehX;
  Real VehY;
  Real VehTheta;
  Real TarX;
  Real TarY;
  Real TarTheta;
  Real SltPt1X;
  Real SltPt1Y;
  Real SltPt2X;
  Real SltPt2Y;
  Real SltPt3X;
  Real SltPt3Y;
  Real SltPt4X;
  Real SltPt4Y;
  Integer SltEntr;
  Integer SltState;
  Real RminDefault;
  Real RminSuccSlot;
  Real VehW;
  Real VehLlF;
  Real VehLfMin;
  Integer PathSegNum;
  Integer PathGoDir[APA_PATH_SEG_MAX];
  Integer PathTrnDir[APA_PATH_SEG_MAX];
  Real PathCrv[APA_PATH_SEG_MAX];
  Real PathTht[APA_PATH_SEG_MAX];
  Real PathLen[APA_PATH_SEG_MAX];
  Real PathStx[APA_PATH_SEG_MAX];
  Real PathSty[APA_PATH_SEG_MAX];
  Real PathStTh[APA_PATH_SEG_MAX];
  Real PathCC[APA_PATH_CC_MAX];
} GeoStp0trnmdfInput_t;

/**
 * @brief stp0转向修正输出
 * @en_name GeoStp0trnmdfOutput
 * @cn_name stp0转向修正输出
 * @tag STRUCT
 * @field name=MdfFlg, type=Integer, unit=1, desc=修正成功标志1成功0失败
 * @field name=PlnChkCnt, type=Integer, unit=1, desc=规划检查次数
 * @field name=PathSegNum, type=Integer, unit=1, desc=修正后路径段数
 * @field name=PathGoDir, type=Array<Integer, [30]>, unit=1, desc=行驶方向数组
 * @field name=PathTrnDir, type=Array<Integer, [30]>, unit=1, desc=转向数组
 * @field name=PathCrv, type=Array<Real, [30]>, unit=1/m, desc=曲率数组
 * @field name=PathTht, type=Array<Real, [30]>, unit=rad, desc=转角数组
 * @field name=PathLen, type=Array<Real, [30]>, unit=m, desc=长度数组
 * @field name=PathStx, type=Array<Real, [30]>, unit=m, desc=起点X数组
 * @field name=PathSty, type=Array<Real, [30]>, unit=m, desc=起点Y数组
 * @field name=PathStTh, type=Array<Real, [30]>, unit=rad, desc=起点航向数组
 * @field name=PathCC, type=Array<Real, [60]>, unit=m, desc=圆心数组
 * @version 1.0
 * @date 2026-03-05
 * @author aide
 */
typedef struct {
  Integer MdfFlg;
  Integer PlnChkCnt;
  Integer PathSegNum;
  Integer PathGoDir[APA_PATH_SEG_MAX];
  Integer PathTrnDir[APA_PATH_SEG_MAX];
  Real PathCrv[APA_PATH_SEG_MAX];
  Real PathTht[APA_PATH_SEG_MAX];
  Real PathLen[APA_PATH_SEG_MAX];
  Real PathStx[APA_PATH_SEG_MAX];
  Real PathSty[APA_PATH_SEG_MAX];
  Real PathStTh[APA_PATH_SEG_MAX];
  Real PathCC[APA_PATH_CC_MAX];
} GeoStp0trnmdfOutput_t;

void geoStp0trnmdf(const GeoStp0trnmdfInput_t *input, GeoStp0trnmdfOutput_t *output);

#endif
