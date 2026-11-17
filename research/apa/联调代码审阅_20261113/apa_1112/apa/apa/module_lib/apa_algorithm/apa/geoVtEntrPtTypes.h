#ifndef GEO_VT_ENTR_PT_TYPES_H
#define GEO_VT_ENTR_PT_TYPES_H

#include "baseTypes.h"
#include "geoAxisTransTypes.h"
#include "geoLineCrossPtTypes.h"

/** @brief 泊车方向：左泊入 */
#ifndef APA_PARK_TYPE_LEFT
#define APA_PARK_TYPE_LEFT 1
#endif

/** @brief 泊车方向：右泊入 */
#ifndef APA_PARK_TYPE_RIGHT
#define APA_PARK_TYPE_RIGHT 2
#endif

/** @brief 周边类型：无开口 */
#ifndef APA_SURD_NONE_OPEN
#define APA_SURD_NONE_OPEN 0
#endif

/** @brief 周边类型：左侧开口 */
#ifndef APA_SURD_LEFT_OPEN
#define APA_SURD_LEFT_OPEN 1
#endif

/** @brief 周边类型：右侧开口 */
#ifndef APA_SURD_RIGHT_OPEN
#define APA_SURD_RIGHT_OPEN 2
#endif

/** @brief 周边类型：双侧开口 */
#ifndef APA_SURD_BOTH_OPEN
#define APA_SURD_BOTH_OPEN 3
#endif

/**
 * @brief VT入口点计算输入
 * @en_name GeoVtEntrPtInput
 * @cn_name VT入口点计算输入
 * @tag STRUCT
 * @field name=SltPt1X, type=Real, unit=m, desc=车位角点1X
 * @field name=SltPt1Y, type=Real, unit=m, desc=车位角点1Y
 * @field name=SltPt2X, type=Real, unit=m, desc=车位角点2X
 * @field name=SltPt2Y, type=Real, unit=m, desc=车位角点2Y
 * @field name=SltPt3X, type=Real, unit=m, desc=车位角点3X
 * @field name=SltPt3Y, type=Real, unit=m, desc=车位角点3Y
 * @field name=SltPt4X, type=Real, unit=m, desc=车位角点4X
 * @field name=SltPt4Y, type=Real, unit=m, desc=车位角点4Y
 * @field name=SltEntr, type=Integer, unit=1, desc=入口边编号1到4
 * @field name=VehX, type=Real, unit=m, desc=车辆X（接口保留）
 * @field name=VehY, type=Real, unit=m, desc=车辆Y（接口保留）
 * @field name=VehTheta, type=Real, unit=rad, desc=车辆航向（接口保留）
 * @field name=TgtX, type=Real, unit=m, desc=目标位X
 * @field name=TgtY, type=Real, unit=m, desc=目标位Y
 * @field name=TgtTheta, type=Real, unit=rad, desc=目标位航向
 * @field name=SurdType, type=Integer, unit=1, desc=周边开口类型0无1左2右3双侧
 * @field name=PrkType, type=Integer, unit=1, desc=泊车方向1左2右
 * @version 1.0
 * @date 2026-03-04
 * @author aide
 */
typedef struct {
  Real SltPt1X;
  Real SltPt1Y;
  Real SltPt2X;
  Real SltPt2Y;
  Real SltPt3X;
  Real SltPt3Y;
  Real SltPt4X;
  Real SltPt4Y;
  Integer SltEntr;
  Real VehX;
  Real VehY;
  Real VehTheta;
  Real TgtX;
  Real TgtY;
  Real TgtTheta;
  Integer SurdType;
  Integer PrkType;
} GeoVtEntrPtInput_t;

/**
 * @brief VT入口点计算输出
 * @en_name GeoVtEntrPtOutput
 * @cn_name VT入口点计算输出
 * @tag STRUCT
 * @field name=EntrPtX, type=Real, unit=m, desc=规划入口点X
 * @field name=EntrPtY, type=Real, unit=m, desc=规划入口点Y
 * @field name=LtWd, type=Real, unit=m, desc=入口边左侧宽度
 * @field name=RtWd, type=Real, unit=m, desc=入口边右侧宽度
 * @field name=SltDepth, type=Real, unit=m, desc=车位深度估计
 * @version 1.0
 * @date 2026-03-04
 * @author aide
 */
typedef struct {
  Real EntrPtX;
  Real EntrPtY;
  Real LtWd;
  Real RtWd;
  Real SltDepth;
} GeoVtEntrPtOutput_t;

void geoVtEntrPt(const GeoVtEntrPtInput_t *input, GeoVtEntrPtOutput_t *output);

#endif
