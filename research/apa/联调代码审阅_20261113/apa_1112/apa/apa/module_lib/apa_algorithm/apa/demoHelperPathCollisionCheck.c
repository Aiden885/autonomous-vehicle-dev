#include "apaMathUtils.h"
#include "demoHelperPathCollisionCheckTypes.h"
#include <math.h>

static Real
demoHelperPathCollisionResolveFrontOffset(const DemoHelperPathCollisionCheckParam_t *param) {
  Real frontOffset;
  Real halfLength;

  frontOffset = param->VehicleFrontOffset;
  halfLength = 0.5f * param->VehicleLength;
  if (frontOffset <= DEMO_HELPER_PATH_COLLISION_EPSILON) {
    frontOffset = halfLength;
  }
  if ((halfLength > DEMO_HELPER_PATH_COLLISION_EPSILON) && (frontOffset > halfLength)) {
    frontOffset = halfLength;
  }
  return frontOffset;
}

static void demoHelperPathCollisionResolveSlot(const DemoHelperPathCollisionCheckInput_t *input,
                                               Real *midFrontX,
                                               Real *midFrontY,
                                               Real *axisLatX,
                                               Real *axisLatY,
                                               Real *axisLonX,
                                               Real *axisLonY,
                                               Real *slotWidth,
                                               Real *slotDepth,
                                               Boolean *slotValid) {
  Real entryVecX;
  Real entryVecY;
  Real entryNorm;
  Real goalVecX;
  Real goalVecY;
  Real goalNorm;
  Real depthValue;

  *midFrontX = 0.5f * (input->SlotP1X + input->SlotP2X);
  *midFrontY = 0.5f * (input->SlotP1Y + input->SlotP2Y);
  entryVecX = input->SlotP2X - input->SlotP1X;
  entryVecY = input->SlotP2Y - input->SlotP1Y;
  entryNorm = apaMathNorm2D(entryVecX, entryVecY);
  *slotValid = false;
  *axisLatX = 0.0f;
  *axisLatY = 0.0f;
  *axisLonX = 0.0f;
  *axisLonY = 0.0f;
  *slotWidth = 0.0f;
  *slotDepth = 0.0f;

  if (entryNorm > DEMO_HELPER_PATH_COLLISION_EPSILON) {
    *axisLatX = entryVecX / entryNorm;
    *axisLatY = entryVecY / entryNorm;
    *slotWidth = entryNorm;
  }

  goalVecX = input->GoalX - *midFrontX;
  goalVecY = input->GoalY - *midFrontY;
  goalNorm = apaMathNorm2D(goalVecX, goalVecY);
  if (goalNorm > DEMO_HELPER_PATH_COLLISION_EPSILON) {
    *axisLonX = goalVecX / goalNorm;
    *axisLonY = goalVecY / goalNorm;
  } else {
    *axisLonX = cosf(input->GoalYaw);
    *axisLonY = sinf(input->GoalYaw);
  }

  depthValue = input->SlotDepth;
  if (depthValue < 0.0f) {
    depthValue = -depthValue;
  }
  *slotDepth = depthValue;
  if ((*slotWidth > DEMO_HELPER_PATH_COLLISION_EPSILON) &&
      (*slotDepth > DEMO_HELPER_PATH_COLLISION_EPSILON)) {
    *slotValid = true;
  }
}

static void demoHelperPathCollisionCheckPose(Real poseX,
                                             Real poseY,
                                             Real poseTheta,
                                             Real midFrontX,
                                             Real midFrontY,
                                             Real axisLatX,
                                             Real axisLatY,
                                             Real axisLonX,
                                             Real axisLonY,
                                             Real slotWidth,
                                             Real slotDepth,
                                             Real vehWidth,
                                             Real vehFrontOffset,
                                             Real *poseClearance,
                                             Boolean *collisionFlg) {
  Real frontCenterX;
  Real frontCenterY;
  Real latX;
  Real latY;
  Real halfVehWidth;
  Real halfSlotWidth;
  Real localLong[2];
  Real localLat[2];
  Integer cornerIdx;

  frontCenterX = poseX + vehFrontOffset * cosf(poseTheta);
  frontCenterY = poseY + vehFrontOffset * sinf(poseTheta);
  latX = -sinf(poseTheta);
  latY = cosf(poseTheta);
  halfVehWidth = 0.5f * vehWidth;
  halfSlotWidth = 0.5f * slotWidth;
  *poseClearance = 1.0e6f;
  *collisionFlg = false;
  cornerIdx = 0;

  while ((cornerIdx < 2) && (*collisionFlg == false)) {
    Real cornerX;
    Real cornerY;
    Real relX;
    Real relY;
    Real longClearance;

    if (cornerIdx == 0) {
      cornerX = frontCenterX + halfVehWidth * latX;
      cornerY = frontCenterY + halfVehWidth * latY;
    } else {
      cornerX = frontCenterX - halfVehWidth * latX;
      cornerY = frontCenterY - halfVehWidth * latY;
    }

    relX = cornerX - midFrontX;
    relY = cornerY - midFrontY;
    localLong[cornerIdx] = relX * axisLonX + relY * axisLonY;
    localLat[cornerIdx] = relX * axisLatX + relY * axisLatY;
    if (fabsf(localLat[cornerIdx]) <= (halfSlotWidth + DEMO_HELPER_PATH_COLLISION_EPSILON)) {
      longClearance = slotDepth - localLong[cornerIdx];
      if (longClearance < *poseClearance) {
        *poseClearance = longClearance;
      }
      if (longClearance < -DEMO_HELPER_PATH_COLLISION_EPSILON) {
        *collisionFlg = true;
      }
    }
    cornerIdx = cornerIdx + 1;
  }

  if (*collisionFlg == false) {
    Real edgeDeltaLong;

    edgeDeltaLong = localLong[1] - localLong[0];
    if (fabsf(edgeDeltaLong) > DEMO_HELPER_PATH_COLLISION_EPSILON) {
      Real ratio;

      ratio = (slotDepth - localLong[0]) / edgeDeltaLong;
      if ((ratio >= 0.0f) && (ratio <= 1.0f)) {
        Real crossLat;

        crossLat = localLat[0] + ratio * (localLat[1] - localLat[0]);
        if (fabsf(crossLat) <= (halfSlotWidth + DEMO_HELPER_PATH_COLLISION_EPSILON)) {
          *collisionFlg = true;
          if (*poseClearance > 0.0f) {
            *poseClearance = 0.0f;
          }
        }
      }
    }
  }
}

/**
 * @brief 对整条离散规划路径做前包络检测，仅约束空车位demo里的底边越界
 * @en_name demoHelperPathCollisionCheck
 * @cn_name 演示辅助路径碰撞检测
 * @type widget_module
 * @config cn_name=车辆长度, en_name=VehicleLength,
 * desc=用于前包络回退的车辆长度, unit=m, type=Real, range=[2,8], default=4.8
 * @config cn_name=车辆宽度, en_name=VehicleWidth,
 * desc=用于前包络检测的车辆宽度, unit=m, type=Real, range=[1,3], default=1.9
 * @config cn_name=前包络偏置, en_name=VehicleFrontOffset,
 * desc=前包络检测中心到车辆参考点的前向偏置, unit=m, type=Real, range=[0,3], default=1.05
 * @config cn_name=安全余量, en_name=SafetyMargin,
 * desc=前包络到底边的最小安全间隙阈值, unit=m, type=Real, range=[0,1], default=0.0
 * @param[IN] Integer PtCount 离散路径点数量
 * @param[IN] Real[100] PthPtX 离散路径点X数组
 * @param[IN] Real[100] PthPtY 离散路径点Y数组
 * @param[IN] Real[100] PthPtTht 离散路径点航向数组
 * @param[IN] Real GoalYaw 车位方向航向角
 * @param[IN] Real GoalX 目标车辆X位置
 * @param[IN] Real GoalY 目标车辆Y位置
 * @param[IN] Real SlotP1X 车位入口角点1X
 * @param[IN] Real SlotP1Y 车位入口角点1Y
 * @param[IN] Real SlotP2X 车位入口角点2X
 * @param[IN] Real SlotP2Y 车位入口角点2Y
 * @param[IN] Real SlotDepth 车位深度
 * @param[OUT] Integer PathCollisionFlg 路径碰撞标志0安全1碰撞或侵入安全余量
 * @param[OUT] Real PathMinClearance 路径最小边界间隙
 * @param[OUT] Real RiskPoseX 风险路径点X
 * @param[OUT] Real RiskPoseY 风险路径点Y
 * @var Integer Reserved 无状态占位字段
 * @granularity atomic
 * @tag_level1 演示
 * @tag_level2 碰撞适配
 * @source demo_new
 * @version 1.0
 * @date 2026-03-08
 * @author aide
 */
void demoHelperPathCollisionCheck(const DemoHelperPathCollisionCheckInput_t *input,
                                  const DemoHelperPathCollisionCheckParam_t *param,
                                  DemoHelperPathCollisionCheckState_t *state,
                                  DemoHelperPathCollisionCheckOutput_t *output) {
  Real midFrontX;
  Real midFrontY;
  Real axisLatX;
  Real axisLatY;
  Real axisLonX;
  Real axisLonY;
  Real slotWidth;
  Real slotDepth;
  Boolean slotValid;
  Real vehWidth;
  Real vehFrontOffset;
  Boolean haveEnvelope;
  Integer ptLimit;
  Integer ptIdx;
  Real minClearance;
  Real riskPoseX;
  Real riskPoseY;
  Boolean collisionFlg;

  (void)state;

  demoHelperPathCollisionResolveSlot(input,
                                     &midFrontX,
                                     &midFrontY,
                                     &axisLatX,
                                     &axisLatY,
                                     &axisLonX,
                                     &axisLonY,
                                     &slotWidth,
                                     &slotDepth,
                                     &slotValid);
  vehWidth = param->VehicleWidth;
  vehFrontOffset = demoHelperPathCollisionResolveFrontOffset(param);
  haveEnvelope = slotValid;
  if (vehWidth <= DEMO_HELPER_PATH_COLLISION_EPSILON) {
    haveEnvelope = false;
  }
  if (vehFrontOffset <= DEMO_HELPER_PATH_COLLISION_EPSILON) {
    haveEnvelope = false;
  }

  ptLimit = input->PtCount;
  if (ptLimit < 0) {
    ptLimit = 0;
  }
  if (ptLimit > DEMO_PATH_POINT_COUNT) {
    ptLimit = DEMO_PATH_POINT_COUNT;
  }

  minClearance = 1.0e6f;
  riskPoseX = 0.0f;
  riskPoseY = 0.0f;
  collisionFlg = false;
  ptIdx = 0;

  while (ptIdx < ptLimit) {
    Real poseClearance;
    Boolean poseCollisionFlg;

    poseClearance = 1.0e6f;
    poseCollisionFlg = false;
    if (haveEnvelope) {
      demoHelperPathCollisionCheckPose(input->PthPtX[ptIdx],
                                       input->PthPtY[ptIdx],
                                       input->PthPtTht[ptIdx],
                                       midFrontX,
                                       midFrontY,
                                       axisLatX,
                                       axisLatY,
                                       axisLonX,
                                       axisLonY,
                                       slotWidth,
                                       slotDepth,
                                       vehWidth,
                                       vehFrontOffset,
                                       &poseClearance,
                                       &poseCollisionFlg);
      if (poseClearance < minClearance) {
        minClearance = poseClearance;
        riskPoseX = input->PthPtX[ptIdx];
        riskPoseY = input->PthPtY[ptIdx];
      }
      if (poseCollisionFlg) {
        collisionFlg = true;
      }
    }

    ptIdx = ptIdx + 1;
  }

  if (ptLimit <= 0) {
    minClearance = 0.0f;
  }
  if (haveEnvelope == false) {
    minClearance = 1.0e6f;
  }
  if (minClearance <= param->SafetyMargin) {
    collisionFlg = true;
  }

  if (collisionFlg) {
    output->PathCollisionFlg = 1;
  } else {
    output->PathCollisionFlg = 0;
  }
  output->PathMinClearance = minClearance;
  output->RiskPoseX = riskPoseX;
  output->RiskPoseY = riskPoseY;
}
