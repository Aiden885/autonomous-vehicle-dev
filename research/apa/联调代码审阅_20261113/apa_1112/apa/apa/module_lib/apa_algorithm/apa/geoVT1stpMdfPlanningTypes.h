#ifndef GEO_VT_1STP_MDF_PLANNING_TYPES_H
#define GEO_VT_1STP_MDF_PLANNING_TYPES_H

#include "baseTypes.h"
#include "geoStp0trnmdfTypes.h"
#include "geoStp1trnmdfTypes.h"
#include "geoRev2CircleLinkTypes.h"
#include "geoCircleIntrvCheckTypes.h"
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
 * @brief VT单步修正主规划输入
 * @en_name GeoVT1stpMdfPlanningInput
 * @cn_name VT单步修正主规划输入
 * @tag STRUCT
 * @field name=VehX, type=Real, unit=m, desc=车辆X
 * @field name=VehY, type=Real, unit=m, desc=车辆Y
 * @field name=VehTheta, type=Real, unit=rad, desc=车辆航向
 * @field name=TarX, type=Real, unit=m, desc=目标X
 * @field name=TarY, type=Real, unit=m, desc=目标Y
 * @field name=TarTheta, type=Real, unit=rad, desc=目标航向
 * @field name=EntrPtX, type=Real, unit=m, desc=修正点X（对应MATLAB MdfPt(1)）
 * @field name=EntrPtY, type=Real, unit=m, desc=修正点Y（对应MATLAB MdfPt(2)）
 * @field name=MdfPtTheta, type=Real, unit=rad, desc=修正点航向（对应MATLAB MdfPt(3)）
 * @field name=SltPt1X, type=Real, unit=m, desc=车位角点1X
 * @field name=SltPt1Y, type=Real, unit=m, desc=车位角点1Y
 * @field name=SltPt2X, type=Real, unit=m, desc=车位角点2X
 * @field name=SltPt2Y, type=Real, unit=m, desc=车位角点2Y
 * @field name=SltPt3X, type=Real, unit=m, desc=车位角点3X
 * @field name=SltPt3Y, type=Real, unit=m, desc=车位角点3Y
 * @field name=SltPt4X, type=Real, unit=m, desc=车位角点4X
 * @field name=SltPt4Y, type=Real, unit=m, desc=车位角点4Y
 * @field name=SltEntr, type=Integer, unit=1, desc=入口边编号1到4
 * @field name=SltState, type=Integer, unit=1, desc=车位状态
 * @field name=RminDefault, type=Real, unit=m, desc=默认最小半径
 * @field name=RminSuccSlot, type=Real, unit=m, desc=成功车位最小半径
 * @field name=VehW, type=Real, unit=m, desc=车宽
 * @field name=VehLlF, type=Real, unit=m, desc=前悬到参考点长度
 * @field name=VehLfMin, type=Real, unit=m, desc=最小前向安全长度
 * @field name=VehLr, type=Real, unit=m, desc=后悬到参考点长度
 * @field name=BoundaryLineNum, type=Integer, unit=1, desc=障碍边界线段数
 * @version 1.1
 * @date 2026-03-14
 * @author aide
 */

#ifndef GEO_BOUNDARY_LINE_MAX
#define GEO_BOUNDARY_LINE_MAX 64
#endif

typedef struct {
  Real VehX;
  Real VehY;
  Real VehTheta;
  Real TarX;
  Real TarY;
  Real TarTheta;
  Real EntrPtX;
  Real EntrPtY;
  Real MdfPtTheta;
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
  Real VehLr;
  Integer BoundaryLineNum;
  Real BoundaryLineX1[GEO_BOUNDARY_LINE_MAX];
  Real BoundaryLineY1[GEO_BOUNDARY_LINE_MAX];
  Real BoundaryLineX2[GEO_BOUNDARY_LINE_MAX];
  Real BoundaryLineY2[GEO_BOUNDARY_LINE_MAX];
} GeoVT1stpMdfPlanningInput_t;

/**
 * @brief VT单步修正主规划输出
 * @en_name GeoVT1stpMdfPlanningOutput
 * @cn_name VT单步修正主规划输出
 * @tag STRUCT
 * @field name=PthPlnFlg, type=Integer, unit=1, desc=规划成功标志
 * @field name=StpPthTypLN, type=Integer, unit=1, desc=路径类型序列长度
 * @field name=StpPthTypList, type=Array<Integer, [30]>, unit=1, desc=路径类型序列
 * @field name=FailRsOut, type=Array<Integer, [30]>, unit=1, desc=失败原因序列
 * @field name=Pt1X, type=Real, unit=m, desc=入口点X
 * @field name=Pt1Y, type=Real, unit=m, desc=入口点Y
 * @field name=PlnChkCnt, type=Integer, unit=1, desc=规划检查次数
 * @field name=PathLen, type=Real, unit=m, desc=路径总长
 * @field name=StpN, type=Integer, unit=1, desc=换向次数
 * @field name=PathSegNum, type=Integer, unit=1, desc=路径段数
 * @field name=PathGoDir, type=Array<Integer, [30]>, unit=1, desc=行驶方向数组
 * @field name=PathTrnDir, type=Array<Integer, [30]>, unit=1, desc=转向数组
 * @field name=PathCrv, type=Array<Real, [30]>, unit=1/m, desc=曲率数组
 * @field name=PathTht, type=Array<Real, [30]>, unit=rad, desc=转角数组
 * @field name=PathLenArr, type=Array<Real, [30]>, unit=m, desc=长度数组
 * @field name=PathStx, type=Array<Real, [30]>, unit=m, desc=起点X数组
 * @field name=PathSty, type=Array<Real, [30]>, unit=m, desc=起点Y数组
 * @field name=PathStTh, type=Array<Real, [30]>, unit=rad, desc=起点航向数组
 * @field name=PathCC, type=Array<Real, [60]>, unit=m, desc=圆心数组
 * @version 1.0
 * @date 2026-03-05
 * @author aide
 */
typedef struct {
  Integer PthPlnFlg;
  Integer StpPthTypLN;
  Integer StpPthTypList[APA_PATH_SEG_MAX];
  Integer FailRsOut[APA_PATH_SEG_MAX];
  Real Pt1X;
  Real Pt1Y;
  Integer PlnChkCnt;
  Real PathLen;
  Integer StpN;
  Integer PathSegNum;
  Integer PathGoDir[APA_PATH_SEG_MAX];
  Integer PathTrnDir[APA_PATH_SEG_MAX];
  Real PathCrv[APA_PATH_SEG_MAX];
  Real PathTht[APA_PATH_SEG_MAX];
  Real PathLenArr[APA_PATH_SEG_MAX];
  Real PathStx[APA_PATH_SEG_MAX];
  Real PathSty[APA_PATH_SEG_MAX];
  Real PathStTh[APA_PATH_SEG_MAX];
  Real PathCC[APA_PATH_CC_MAX];
} GeoVT1stpMdfPlanningOutput_t;

void geoVT1stpMdfPlanning(const GeoVT1stpMdfPlanningInput_t *input,
                          GeoVT1stpMdfPlanningOutput_t *output);

#endif
