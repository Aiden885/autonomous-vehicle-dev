#ifndef GEO_PTH_SHORT_CUT_TYPES_H
#define GEO_PTH_SHORT_CUT_TYPES_H

#include "baseTypes.h"

/** @brief 路径点最大数量 */
#ifndef PATH_PT_MAX
#define PATH_PT_MAX 100
#endif

/**
 * @brief 路径shortcut输入
 * @en_name GeoPthShortCutInput
 * @cn_name 路径shortcut输入
 * @tag STRUCT
 * @field name=ShortCutSw, type=Integer, unit=1, desc=是否启用shortcut
 * @field name=SegNum, type=Integer, unit=1, desc=路径段数
 * @field name=Seg1TrnDir, type=Integer, unit=1, desc=第1段转向
 * @field name=Seg2TrnDir, type=Integer, unit=1, desc=第2段转向
 * @field name=Seg1GoDir, type=Integer, unit=1, desc=第1段行驶方向
 * @field name=Seg2GoDir, type=Integer, unit=1, desc=第2段行驶方向
 * @field name=Seg1Len, type=Real, unit=m, desc=第1段长度
 * @field name=Seg2Len, type=Real, unit=m, desc=第2段长度
 * @field name=Stp2Idx, type=Integer, unit=1, desc=第2段起始索引
 * @field name=PtCountIn, type=Integer, unit=1, desc=输入点数
 * @field name=PthPtXIn, type=Array<Real, [100]>, unit=m, desc=输入轨迹X
 * @field name=PthPtYIn, type=Array<Real, [100]>, unit=m, desc=输入轨迹Y
 * @field name=PthPtThtIn, type=Array<Real, [100]>, unit=rad, desc=输入轨迹航向
 * @field name=PthPtOdoIn, type=Array<Real, [100]>, unit=m, desc=输入轨迹里程
 * @field name=VehPosX, type=Real, unit=m, desc=车辆X
 * @field name=VehPosY, type=Real, unit=m, desc=车辆Y
 * @field name=VehPosTheta, type=Real, unit=rad, desc=车辆航向
 * @field name=ShortCutThd, type=Real, unit=m, desc=shortcut横向阈值
 * @version 1.0
 * @date 2026-03-04
 * @author aide
 */
typedef struct {
  Integer ShortCutSw;
  Integer SegNum;
  Integer Seg1TrnDir;
  Integer Seg2TrnDir;
  Integer Seg1GoDir;
  Integer Seg2GoDir;
  Real Seg1Len;
  Real Seg2Len;
  Integer Stp2Idx;
  Integer PtCountIn;
  Real PthPtXIn[PATH_PT_MAX];
  Real PthPtYIn[PATH_PT_MAX];
  Real PthPtThtIn[PATH_PT_MAX];
  Real PthPtOdoIn[PATH_PT_MAX];
  Real VehPosX;
  Real VehPosY;
  Real VehPosTheta;
  Real ShortCutThd;
} GeoPthShortCutInput_t;

/**
 * @brief 路径shortcut输出
 * @en_name GeoPthShortCutOutput
 * @cn_name 路径shortcut输出
 * @tag STRUCT
 * @field name=PtCount, type=Integer, unit=1, desc=输出点数
 * @field name=CutApplied, type=Integer, unit=1, desc=是否执行cut
 * @field name=PthPtX, type=Array<Real, [100]>, unit=m, desc=输出轨迹X
 * @field name=PthPtY, type=Array<Real, [100]>, unit=m, desc=输出轨迹Y
 * @field name=PthPtTht, type=Array<Real, [100]>, unit=rad, desc=输出轨迹航向
 * @field name=PthPtOdo, type=Array<Real, [100]>, unit=m, desc=输出轨迹里程
 * @version 1.0
 * @date 2026-03-04
 * @author aide
 */
typedef struct {
  Integer PtCount;
  Integer CutApplied;
  Real PthPtX[PATH_PT_MAX];
  Real PthPtY[PATH_PT_MAX];
  Real PthPtTht[PATH_PT_MAX];
  Real PthPtOdo[PATH_PT_MAX];
} GeoPthShortCutOutput_t;

void geoPthShortCut(const GeoPthShortCutInput_t *input, GeoPthShortCutOutput_t *output);

#endif
