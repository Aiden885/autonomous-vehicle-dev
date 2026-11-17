#ifndef GEO_PTH_DISCRET_HT_TYPES_H
#define GEO_PTH_DISCRET_HT_TYPES_H

#include "baseTypes.h"

/** @brief 路径最大段数 */
#define PATH_SEG_MAX 30
/** @brief 路径点最大数量 */
#ifndef PATH_PT_MAX
#define PATH_PT_MAX 100
#endif
/** @brief 圆周率 */
#ifndef APA_PI
#define APA_PI 3.14159265f
#endif
/** @brief 除零保护精度 */
#ifndef GEO_EPSILON
#define GEO_EPSILON 1e-6f
#endif
/** @brief 输出段最大数量 */
#ifndef HT_SEG_OUT_MAX
#define HT_SEG_OUT_MAX 30
#endif

/**
 * @brief HT路径离散化输入
 * @en_name GeoPthDiscretHtInput
 * @cn_name HT路径离散化输入
 * @tag STRUCT
 * @field name=VehPosX, type=Real, unit=m, desc=车辆X
 * @field name=VehPosY, type=Real, unit=m, desc=车辆Y
 * @field name=VehPosTheta, type=Real, unit=rad, desc=车辆航向
 * @field name=TarPtX, type=Real, unit=m, desc=目标X
 * @field name=TarPtY, type=Real, unit=m, desc=目标Y
 * @field name=TarPtTheta, type=Real, unit=rad, desc=目标航向
 * @field name=PathLen, type=Real, unit=m, desc=路径总长
 * @field name=SegNum, type=Integer, unit=1, desc=路径段数
 * @field name=SegGoDir, type=Array<Integer, [30]>, unit=1, desc=各段行驶方向
 * @field name=SegTrnDir, type=Array<Integer, [30]>, unit=1, desc=各段转弯方向
 * @field name=SegCrv, type=Array<Real, [30]>, unit=1/m, desc=各段曲率
 * @field name=SegTht, type=Array<Real, [30]>, unit=rad, desc=各段圆弧角
 * @field name=SegLen, type=Array<Real, [30]>, unit=m, desc=各段长度
 * @field name=SegStX, type=Array<Real, [30]>, unit=m, desc=各段起点X
 * @field name=SegStY, type=Array<Real, [30]>, unit=m, desc=各段起点Y
 * @field name=SegStTh, type=Array<Real, [30]>, unit=rad, desc=各段起点航向
 * @field name=SegCcX, type=Array<Real, [30]>, unit=m, desc=各段圆心X
 * @field name=SegCcY, type=Array<Real, [30]>, unit=m, desc=各段圆心Y
 * @field name=ShortCutSw, type=Integer, unit=1, desc=是否启用轨迹shortcut
 * @field name=ShortCutThd, type=Real, unit=m, desc=shortcut横向阈值
 * @version 1.1
 * @date 2026-03-04
 * @author aide
 */
typedef struct {
  Real VehPosX;
  Real VehPosY;
  Real VehPosTheta;
  Real TarPtX;
  Real TarPtY;
  Real TarPtTheta;
  Real PathLen;
  Integer SegNum;
  Integer SegGoDir[HT_SEG_OUT_MAX];
  Integer SegTrnDir[HT_SEG_OUT_MAX];
  Real SegCrv[HT_SEG_OUT_MAX];
  Real SegTht[HT_SEG_OUT_MAX];
  Real SegLen[HT_SEG_OUT_MAX];
  Real SegStX[HT_SEG_OUT_MAX];
  Real SegStY[HT_SEG_OUT_MAX];
  Real SegStTh[HT_SEG_OUT_MAX];
  Real SegCcX[HT_SEG_OUT_MAX];
  Real SegCcY[HT_SEG_OUT_MAX];
  Integer ShortCutSw;
  Real ShortCutThd;
} GeoPthDiscretHtInput_t;

/**
 * @brief HT路径离散化输出
 * @en_name GeoPthDiscretHtOutput
 * @cn_name HT路径离散化输出
 * @tag STRUCT
 * @field name=PtCount, type=Integer, unit=1, desc=离散点数
 * @field name=StepCount, type=Integer, unit=1, desc=MATLAB PthPtStp机动步数，初值1且仅换向加1
 * @field name=St2Idx, type=Integer, unit=1, desc=MATLAB 1-based首段结束点序号，0表示不可用
 * @field name=Stp1PtRX, type=Real, unit=m, desc=固定为0
 * @field name=Stp1PtRY, type=Real, unit=m, desc=固定为0
 * @field name=Stp1PtRTheta, type=Real, unit=rad, desc=固定为0
 * @field name=PthPtX, type=Array<Real, [100]>, unit=m, desc=离散点X数组
 * @field name=PthPtY, type=Array<Real, [100]>, unit=m, desc=离散点Y数组
 * @field name=PthPtTht, type=Array<Real, [100]>, unit=rad, desc=离散点航向数组
 * @field name=PthPtOdo, type=Array<Real, [100]>, unit=m, desc=离散点里程数组
 * @version 1.1
 * @date 2026-03-04
 * @author aide
 */
typedef struct {
  Integer PtCount;
  Integer StepCount;
  Integer St2Idx;
  Real Stp1PtRX;
  Real Stp1PtRY;
  Real Stp1PtRTheta;
  Real PthPtX[PATH_PT_MAX];
  Real PthPtY[PATH_PT_MAX];
  Real PthPtTht[PATH_PT_MAX];
  Real PthPtOdo[PATH_PT_MAX];
} GeoPthDiscretHtOutput_t;

void geoPthDiscretHt(const GeoPthDiscretHtInput_t *input, GeoPthDiscretHtOutput_t *output);

/* HtPkIn variant: identical interface to geoPthDiscretHt but uses MATLAB
 * geo_PthDiscret_HtPkIn.m's 0.12 minimum step length instead of 0.15. */
void geoPthDiscretHtPkIn(const GeoPthDiscretHtInput_t *input, GeoPthDiscretHtOutput_t *output);

#endif
