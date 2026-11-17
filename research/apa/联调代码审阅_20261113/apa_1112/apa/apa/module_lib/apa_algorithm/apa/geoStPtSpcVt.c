#include "geoStPtSpcVtTypes.h"
#include "apaMathUtils.h"
#include <math.h>

#ifndef GEO_ST_PT_SPC_VT_EPSILON
#define GEO_ST_PT_SPC_VT_EPSILON 1e-5f
#endif

#ifndef GEO_ST_PT_SPC_VT_MAX_DEFAULT
#define GEO_ST_PT_SPC_VT_MAX_DEFAULT 4.1f
#endif

#ifndef GEO_ST_PT_SPC_VT_FRONT_BUFFER
#define GEO_ST_PT_SPC_VT_FRONT_BUFFER 1.5f
#endif

#ifndef GEO_ST_PT_SPC_VT_BACK_BUFFER
#define GEO_ST_PT_SPC_VT_BACK_BUFFER 0.6f
#endif

#ifndef GEO_ST_PT_SPC_VT_STEP
#define GEO_ST_PT_SPC_VT_STEP 0.8f
#endif

#ifndef GEO_ST_PT_SPC_VT_LAT_MAX
#define GEO_ST_PT_SPC_VT_LAT_MAX 4.1f
#endif

static void geoStPtSpcVtResolveAxes(const GeoStPtSpcVtInput_t *input,
                                    Real *midFrontX,
                                    Real *midFrontY,
                                    Real *axisLatX,
                                    Real *axisLatY,
                                    Real *axisLonX,
                                    Real *axisLonY,
                                    Real *slotWidth,
                                    Real *slotDepth) {
  Real p12X;
  Real p12Y;
  Real p12Norm;
  Real midBackX;
  Real midBackY;
  Real midDeltaX;
  Real midDeltaY;
  Real midNorm;

  *midFrontX = 0.5f * (input->SltPt1X + input->SltPt2X);
  *midFrontY = 0.5f * (input->SltPt1Y + input->SltPt2Y);
  p12X = input->SltPt2X - input->SltPt1X;
  p12Y = input->SltPt2Y - input->SltPt1Y;
  p12Norm = apaMathNorm2D(p12X, p12Y);

  if (p12Norm > GEO_ST_PT_SPC_VT_EPSILON) {
    *axisLatX = p12X / p12Norm;
    *axisLatY = p12Y / p12Norm;
    *slotWidth = p12Norm;
  } else {
    *axisLonX = cosf(input->TarPtTheta);
    *axisLonY = sinf(input->TarPtTheta);
    *axisLatX = -*axisLonY;
    *axisLatY = *axisLonX;
    *slotWidth = GEO_ST_PT_SPC_VT_MAX_DEFAULT;
  }

  midBackX = 0.5f * (input->SltPt3X + input->SltPt4X);
  midBackY = 0.5f * (input->SltPt3Y + input->SltPt4Y);
  midDeltaX = midBackX - *midFrontX;
  midDeltaY = midBackY - *midFrontY;
  midNorm = apaMathNorm2D(midDeltaX, midDeltaY);

  if (midNorm > GEO_ST_PT_SPC_VT_EPSILON) {
    *axisLonX = midDeltaX / midNorm;
    *axisLonY = midDeltaY / midNorm;
    *slotDepth = midNorm;
  } else {
    *axisLonX = cosf(input->TarPtTheta);
    *axisLonY = sinf(input->TarPtTheta);
    *slotDepth = 0.0f;
  }
}

static void geoStPtSpcVtCorridorEstimate(const GeoStPtSpcVtInput_t *input,
                                         GeoStPtSpcVtOutput_t *output) {
  Real maxLatSpace;
  Real midFrontX;
  Real midFrontY;
  Real axisLatX;
  Real axisLatY;
  Real axisLonX;
  Real axisLonY;
  Real slotWidth;
  Real slotDepth;
  Real checkX;
  Real checkY;
  Real relX;
  Real relY;
  Real localLong;
  Real localLat;
  Real halfWidth;
  Real leftLimit;
  Real rightLimit;
  Boolean hasDepth;
  Boolean inCorridor;

  maxLatSpace = input->FallbackSpace;
  if (maxLatSpace < GEO_ST_PT_SPC_VT_EPSILON) {
    maxLatSpace = GEO_ST_PT_SPC_VT_MAX_DEFAULT;
  }
  geoStPtSpcVtResolveAxes(input,
                          &midFrontX,
                          &midFrontY,
                          &axisLatX,
                          &axisLatY,
                          &axisLonX,
                          &axisLonY,
                          &slotWidth,
                          &slotDepth);
  hasDepth = (slotDepth > GEO_ST_PT_SPC_VT_EPSILON);
  checkX = input->VehPosX + input->CheckLen * cosf(input->VehPosTheta);
  checkY = input->VehPosY + input->CheckLen * sinf(input->VehPosTheta);
  relX = checkX - midFrontX;
  relY = checkY - midFrontY;
  localLong = relX * axisLonX + relY * axisLonY;
  localLat = relX * axisLatX + relY * axisLatY;
  halfWidth = 0.5f * slotWidth;
  inCorridor = (localLong >= -GEO_ST_PT_SPC_VT_FRONT_BUFFER) &&
               (localLong <= (slotDepth + GEO_ST_PT_SPC_VT_BACK_BUFFER));

  if ((!hasDepth) || (!inCorridor)) {
    output->LeftSpace = maxLatSpace;
    output->RightSpace = maxLatSpace;
    output->CheckCount = 1;
    output->IntrvRs = 0;
    output->CrashPosX = input->VehPosX;
    output->CrashPosY = input->VehPosY;
    output->CrashPosTheta = input->VehPosTheta;
    output->CorridorValid = 0;
  } else {
    leftLimit = halfWidth - localLat;
    rightLimit = halfWidth + localLat;
    output->LeftSpace = apaMathClamp(leftLimit, 0.0f, maxLatSpace);
    output->RightSpace = apaMathClamp(rightLimit, 0.0f, maxLatSpace);
    output->CheckCount = 1;
    output->IntrvRs = 0;
    output->CrashPosX = checkX;
    output->CrashPosY = checkY;
    output->CrashPosTheta = input->VehPosTheta;
    output->CorridorValid = 1;
    if (leftLimit < 0.0f) {
      output->IntrvRs = 1;
      output->CrashPosX = checkX + leftLimit * axisLatX;
      output->CrashPosY = checkY + leftLimit * axisLatY;
    }
    if (rightLimit < 0.0f) {
      output->IntrvRs = 2;
      output->CrashPosX = checkX - rightLimit * axisLatX;
      output->CrashPosY = checkY - rightLimit * axisLatY;
    }
  }
}

static Boolean geoStPtSpcVtRunCheck(const VehicleCollisionCheckInput_t *tmpl,
                                    const ApaPlanningSceneQuery_t *scene,
                                    Real chkX,
                                    Real chkY,
                                    Real chkTheta,
                                    Integer *collisionType) {
  VehicleCollisionCheckInput_t scratch;
  VehicleCollisionCheckOutput_t result;

  scratch = *tmpl;
  scratch.VehPosX = chkX;
  scratch.VehPosY = chkY;
  scratch.VehPosTheta = chkTheta;

  scratch.CheckType = VCC_CHECK_TYPE_PLANNING;
  if (scratch.CollisionDemand != VCC_COLLISION_DEMAND_ALL) {
    scratch.CollisionDemand = VCC_COLLISION_DEMAND_SIMPLIFY;
  }
  result.IsCollision = false;
  result.CollisionType = 0;
  apaPlanningSceneCollisionCheck(&scratch, scene, &result);
  *collisionType = result.CollisionType;
  return result.IsCollision;
}

static void geoStPtSpcVtScanSide(const VehicleCollisionCheckInput_t *tmpl,
                                 const ApaPlanningSceneQuery_t *scene,
                                 Real centerX,
                                 Real centerY,
                                 Real centerTheta,
                                 Real sideSign,
                                 Real *space,
                                 Integer *intrvRs,
                                 Real *crshX,
                                 Real *crshY,
                                 Real *crshTheta,
                                 Integer *checkCount) {
  Real sinT;
  Real cosT;
  Real sfLen;
  Real lenTmp;
  Real chkX;
  Real chkY;
  Real chkTheta;
  Real avlX;
  Real avlY;
  Real avlTheta;
  Boolean isCollision;
  Boolean sweepOpen;
  Integer collisionType;

  sinT = sinf(centerTheta);
  cosT = cosf(centerTheta);

  sfLen = 0.0f;
  lenTmp = 0.0f;
  chkX = centerX;
  chkY = centerY;
  chkTheta = centerTheta;
  avlX = centerX;
  avlY = centerY;
  avlTheta = centerTheta;
  isCollision = false;
  collisionType = 0;
  sweepOpen = (lenTmp <= GEO_ST_PT_SPC_VT_LAT_MAX);

  while (sweepOpen) {
    chkX = centerX + sideSign * (-sinT) * lenTmp;
    chkY = centerY + sideSign * (cosT)*lenTmp;
    chkTheta = centerTheta;
    isCollision = geoStPtSpcVtRunCheck(tmpl, scene, chkX, chkY, chkTheta, &collisionType);
    *checkCount = *checkCount + 1;
    if (isCollision) {
      *intrvRs = collisionType;
      *crshX = chkX;
      *crshY = chkY;
      *crshTheta = chkTheta;
      sweepOpen = false;
    } else {
      sfLen = lenTmp;
      avlX = chkX;
      avlY = chkY;
      avlTheta = chkTheta;
      lenTmp = lenTmp + GEO_ST_PT_SPC_VT_STEP;
      sweepOpen = (lenTmp <= GEO_ST_PT_SPC_VT_LAT_MAX);
    }
  }

  if (!isCollision) {
    *space = sfLen;
  } else if (lenTmp < (0.0f + 1e-9f)) {
    *space = 0.0f;
  } else {

    Real lenPrcs;
    Real stX;
    Real stY;
    Real stTheta;
    Real edX;
    Real edY;
    Real edTheta;
    Real lenAcc;

    lenPrcs = GEO_ST_PT_SPC_VT_STEP * 0.5f;
    stX = avlX;
    stY = avlY;
    stTheta = avlTheta;
    edX = chkX;
    edY = chkY;
    edTheta = chkTheta;
    lenAcc = 0.0f;

    while ((double)lenPrcs > 0.05) {
      Real midX;
      Real midY;
      Real midTheta;
      Boolean midCollision;
      Integer midType;

      midX = (stX + edX) * 0.5f;
      midY = (stY + edY) * 0.5f;
      midTheta = (stTheta + edTheta) * 0.5f;
      midCollision = geoStPtSpcVtRunCheck(tmpl, scene, midX, midY, midTheta, &midType);
      *checkCount = *checkCount + 1;
      if (midCollision) {
        edX = midX;
        edY = midY;
        edTheta = midTheta;
        *intrvRs = midType;
      } else {
        stX = midX;
        stY = midY;
        stTheta = midTheta;
        lenAcc = lenAcc + lenPrcs;
      }
      lenPrcs = lenPrcs * 0.5f;
    }

    *space = sfLen + lenAcc;
    *crshX = edX;
    *crshY = edY;
    *crshTheta = edTheta;
  }
}

/**
 * @brief VT起点横向空间；UseCollisionCheck 启用时扫描精度为0.05m，否则按槽位走廊估算。
 * @en_name geoStPtSpcVt
 * @cn_name VT起点空间检测
 * @type module
 * @param[IN] Real VehPosX 车辆位置X
 * @param[IN] Real VehPosY 车辆位置Y
 * @param[IN] Real VehPosTheta 车辆航向角
 * @param[IN] Real TarPtX 目标点X
 * @param[IN] Real TarPtY 目标点Y
 * @param[IN] Real TarPtTheta 目标点航向角
 * @param[IN] Real SltPt1X 车位角点1X
 * @param[IN] Real SltPt1Y 车位角点1Y
 * @param[IN] Real SltPt2X 车位角点2X
 * @param[IN] Real SltPt2Y 车位角点2Y
 * @param[IN] Real SltPt3X 车位角点3X
 * @param[IN] Real SltPt3Y 车位角点3Y
 * @param[IN] Real SltPt4X 车位角点4X
 * @param[IN] Real SltPt4Y 车位角点4Y
 * @param[IN] Integer SltEntr [RESERVED] 车位入口边编号
 * @param[IN] Real CheckLen 前向检查距离
 * @param[IN] Integer SltType [RESERVED] 车位类型
 * @param[IN] Real FallbackSpace 最大横向空间回退值
 * @param[IN] Boolean UseCollisionCheck 是否走逐点碰撞扫描
 * @param[IN] VehicleCollisionCheckInput_t VccTemplate 碰撞检测模板含障碍输入
 * @param[OUT] Real LeftSpace 左侧可用空间
 * @param[OUT] Real RightSpace 右侧可用空间
 * @param[OUT] Integer CheckCount 检查次数
 * @param[OUT] Integer IntrvRs 干涉原因
 * @param[OUT] Real CrashPosX 碰撞位置X
 * @param[OUT] Real CrashPosY 碰撞位置Y
 * @param[OUT] Real CrashPosTheta 碰撞位置航向
 * @param[OUT] Integer CorridorValid 是否进入槽位走廊或扫描成功
 * @granularity composite
 * @tag_level1 泊车规划
 * @tag_level2 碰撞检测
 * @source matlab_rewrite geo_StPtSpc_vt.m
 * @version 2.0
 * @date 2026-05-01
 * @author aide
 */
void geoStPtSpcVt(const GeoStPtSpcVtInput_t *input, GeoStPtSpcVtOutput_t *output) {
  geoStPtSpcVtWithScene(input, (const ApaPlanningSceneQuery_t *)0, output);
}

void geoStPtSpcVtWithScene(const GeoStPtSpcVtInput_t *input,
                           const ApaPlanningSceneQuery_t *scene,
                           GeoStPtSpcVtOutput_t *output) {
  if (!input->UseCollisionCheck) {
    geoStPtSpcVtCorridorEstimate(input, output);
  }

  if (input->UseCollisionCheck) {

    Real chkCx;
    Real chkCy;
    Real chkCtheta;
    Real maxLatSpace;
    Integer checkCount;
    Real leftSpc;
    Real rightSpc;
    Integer leftIntrv;
    Integer rightIntrv;
    Real crshX;
    Real crshY;
    Real crshTheta;

    chkCx = input->VehPosX + input->CheckLen * cosf(input->VehPosTheta);
    chkCy = input->VehPosY + input->CheckLen * sinf(input->VehPosTheta);
    chkCtheta = input->VehPosTheta;

    maxLatSpace = input->FallbackSpace;
    if (maxLatSpace < GEO_ST_PT_SPC_VT_EPSILON) {
      maxLatSpace = GEO_ST_PT_SPC_VT_MAX_DEFAULT;
    }

    checkCount = 0;
    leftSpc = 0.0f;
    rightSpc = 0.0f;
    leftIntrv = 0;
    rightIntrv = 0;
    crshX = chkCx;
    crshY = chkCy;
    crshTheta = chkCtheta;

    geoStPtSpcVtScanSide(&input->VccTemplate,
                         scene,
                         chkCx,
                         chkCy,
                         chkCtheta,
                         +1.0f,
                         &leftSpc,
                         &leftIntrv,
                         &crshX,
                         &crshY,
                         &crshTheta,
                         &checkCount);

    Real rcrshX;
    Real rcrshY;
    Real rcrshTheta;
    rcrshX = chkCx;
    rcrshY = chkCy;
    rcrshTheta = chkCtheta;
    geoStPtSpcVtScanSide(&input->VccTemplate,
                         scene,
                         chkCx,
                         chkCy,
                         chkCtheta,
                         -1.0f,
                         &rightSpc,
                         &rightIntrv,
                         &rcrshX,
                         &rcrshY,
                         &rcrshTheta,
                         &checkCount);

    if (leftSpc > maxLatSpace) {
      leftSpc = maxLatSpace;
    }
    if (rightSpc > maxLatSpace) {
      rightSpc = maxLatSpace;
    }

    output->LeftSpace = leftSpc;
    output->RightSpace = rightSpc;
    output->CheckCount = checkCount;

    if (rightIntrv != 0) {
      output->IntrvRs = rightIntrv;
      output->CrashPosX = rcrshX;
      output->CrashPosY = rcrshY;
      output->CrashPosTheta = rcrshTheta;
    } else if (leftIntrv != 0) {
      output->IntrvRs = leftIntrv;
      output->CrashPosX = crshX;
      output->CrashPosY = crshY;
      output->CrashPosTheta = crshTheta;
    } else {
      output->IntrvRs = 0;
      output->CrashPosX = 0.0f;
      output->CrashPosY = 0.0f;
      output->CrashPosTheta = 0.0f;
    }
    output->CorridorValid = 1;
  }
}
