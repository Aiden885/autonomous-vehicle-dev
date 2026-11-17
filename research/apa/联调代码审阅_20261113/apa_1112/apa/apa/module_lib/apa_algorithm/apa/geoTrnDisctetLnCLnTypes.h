#ifndef GEO_TRN_DISCTET_LN_C_LN_TYPES_H
#define GEO_TRN_DISCTET_LN_C_LN_TYPES_H

#include "baseTypes.h"

#ifndef PATH_PT_MAX
#define PATH_PT_MAX 100
#endif

#ifndef APA_PI
#define APA_PI 3.14159265f
#endif

/**
 * @brief 直线-圆弧-直线复合段离散输入
 * @en_name GeoTrnDisctetLnCLnInput
 * @cn_name 直圆直复合段离散化输入
 * @tag STRUCT
 * @field name=Crv, type=Real, unit=1/m, desc=输入段曲率
 * @field name=GoDir, type=Integer, unit=1, desc=行驶方向1前进负1后退
 * @field name=TrnDir, type=Integer, unit=1, desc=转弯方向负1右转1左转
 * @field name=StepLen, type=Real, unit=m, desc=离散步长
 * @field name=ArcAngle, type=Real, unit=rad, desc=输入段圆弧角度
 * @field name=StPtX, type=Real, unit=m, desc=输入段起点X
 * @field name=StPtY, type=Real, unit=m, desc=输入段起点Y
 * @field name=StPtTheta, type=Real, unit=rad, desc=输入段起点航向
 * @field name=EdPtX, type=Real, unit=m, desc=输入段终点X
 * @field name=EdPtY, type=Real, unit=m, desc=输入段终点Y
 * @field name=EdPtTheta, type=Real, unit=rad, desc=输入段终点航向
 * @field name=LstStp, type=Integer, unit=1, desc=末段标志1最后一段
 * @field name=Rmin, type=Real, unit=m, desc=最小转弯半径
 * @field name=VehPosX, type=Real, unit=m, desc=车辆位姿X
 * @field name=VehPosY, type=Real, unit=m, desc=车辆位姿Y
 * @field name=VehPosTheta, type=Real, unit=rad, desc=车辆位姿航向
 * @field name=SltTyp, type=Integer, unit=1, desc=车位类型0垂直1平行
 * @field name=TarPtX, type=Real, unit=m, desc=目标点X
 * @field name=TarPtY, type=Real, unit=m, desc=目标点Y
 * @field name=TarPtTheta, type=Real, unit=rad, desc=目标点航向
 * @field name=LstPtX, type=Real, unit=m, desc=上一离散点X
 * @field name=LstPtY, type=Real, unit=m, desc=上一离散点Y
 * @field name=LstPtTheta, type=Real, unit=rad, desc=上一离散点航向
 * @field name=InPtCount, type=Integer, unit=1, desc=进入时已有点数
 * @field name=InPtX, type=Array<Real, [100]>, unit=m, desc=进入时点X数组
 * @field name=InPtY, type=Array<Real, [100]>, unit=m, desc=进入时点Y数组
 * @field name=InPtTht, type=Array<Real, [100]>, unit=rad, desc=进入时点航向数组
 * @field name=InPtOdo, type=Array<Real, [100]>, unit=m, desc=进入时点里程数组
 * @version 1.0
 * @date 2026-05-01
 * @author aide
 */
typedef struct {
  Real Crv;
  Integer GoDir;
  Integer TrnDir;
  Real StepLen;
  Real ArcAngle;
  Real StPtX;
  Real StPtY;
  Real StPtTheta;
  Real EdPtX;
  Real EdPtY;
  Real EdPtTheta;
  Integer LstStp;
  Real Rmin;
  Real VehPosX;
  Real VehPosY;
  Real VehPosTheta;
  Integer SltTyp;
  Real TarPtX;
  Real TarPtY;
  Real TarPtTheta;
  Real LstPtX;
  Real LstPtY;
  Real LstPtTheta;
  Integer InPtCount;
  Real InPtX[PATH_PT_MAX];
  Real InPtY[PATH_PT_MAX];
  Real InPtTht[PATH_PT_MAX];
  Real InPtOdo[PATH_PT_MAX];
} GeoTrnDisctetLnCLnInput_t;

/**
 * @brief 直线-圆弧-直线复合段离散输出
 * @en_name GeoTrnDisctetLnCLnOutput
 * @cn_name 直圆直复合段离散化输出
 * @tag STRUCT
 * @field name=SuccFlg, type=Integer, unit=1, desc=成功标志0失败1成功
 * @field name=PtCount, type=Integer, unit=1, desc=有效点数
 * @field name=PthPtX, type=Array<Real, [100]>, unit=m, desc=离散点X数组
 * @field name=PthPtY, type=Array<Real, [100]>, unit=m, desc=离散点Y数组
 * @field name=PthPtTht, type=Array<Real, [100]>, unit=rad, desc=离散点航向数组
 * @field name=PthPtOdo, type=Array<Real, [100]>, unit=m, desc=离散点里程数组
 * @field name=LstPtX, type=Real, unit=m, desc=最后离散点X
 * @field name=LstPtY, type=Real, unit=m, desc=最后离散点Y
 * @field name=LstPtTheta, type=Real, unit=rad, desc=最后离散点航向
 * @version 1.0
 * @date 2026-05-01
 * @author aide
 */
typedef struct {
  Integer SuccFlg;
  Integer PtCount;
  Real PthPtX[PATH_PT_MAX];
  Real PthPtY[PATH_PT_MAX];
  Real PthPtTht[PATH_PT_MAX];
  Real PthPtOdo[PATH_PT_MAX];
  Real LstPtX;
  Real LstPtY;
  Real LstPtTheta;
} GeoTrnDisctetLnCLnOutput_t;

void geoTrnDisctetLnCLn(const GeoTrnDisctetLnCLnInput_t *input, GeoTrnDisctetLnCLnOutput_t *output);

#endif
