#include "apaCoordUtils.h"
#include "apaVehicleDefaults.h"
#include "apaMathUtils.h"
#include "apaSlotTypes.h"
#include "apaVehicleUtils.h"
#include "checkParkingTypeTypes.h"
#include <math.h>

#ifndef PARKING_TYPE_MEANINGLESS
#define PARKING_TYPE_MEANINGLESS 0
#endif
#ifndef PARKING_TYPE_LEFT
#define PARKING_TYPE_LEFT 1
#endif
#ifndef PARKING_TYPE_RIGHT
#define PARKING_TYPE_RIGHT 2
#endif
#ifndef PARKING_TYPE_UNKNOWN
#define PARKING_TYPE_UNKNOWN 3
#endif

#ifndef SLOT_CLASS_VERTICAL
#define SLOT_CLASS_VERTICAL 0
#endif
#ifndef SLOT_CLASS_PARALLEL
#define SLOT_CLASS_PARALLEL 1
#endif

#ifndef CHECK_PARKING_TYPE_EPSILON
#define CHECK_PARKING_TYPE_EPSILON 1e-6f
#endif

#ifndef CHECK_PARKING_TYPE_LATERAL_THRESH
#define CHECK_PARKING_TYPE_LATERAL_THRESH 0.3f
#endif

#ifndef CHECK_PARKING_TYPE_YAW_THRESH
#define CHECK_PARKING_TYPE_YAW_THRESH 0.26179939f
#endif

#ifndef CHECK_PARKING_TYPE_DEFAULT_WHEELBASE
#define CHECK_PARKING_TYPE_DEFAULT_WHEELBASE APA_VEHICLE_WHEELBASE_M
#endif

/**
 * @brief 内联展开 ClassSlot.slotPtsPtrOverflowProcess
 *        MATLAB slot_pts 是 1*8 数组(4个角点的xy交叉存储)。
 *        ptr 是 1-based index，溢出时 wrap 到 [1,8]。
 *        C 版本返回 0-based index。
 */
static Integer checkParkingTypeWrapIdx(Integer ptr1based) {
  Integer wrapped0;

  wrapped0 = ptr1based;
  while (wrapped0 > 8) {
    wrapped0 = wrapped0 - 8;
  }

  wrapped0 = wrapped0 - 1;
  return wrapped0;
}

/**
 * @brief 从 4 个角点(8个坐标)中按 EnterEdge 索引取出角点
 *        slot_pts[8] = {pt1x, pt1y, pt2x, pt2y, pt3x, pt3y, pt4x, pt4y}
 */
static Real checkParkingTypeGetPtCoord(const CheckParkingTypeInput_t *input, Integer idx0based) {
  Real coords[8];
  Real result0;

  coords[0] = input->SltPt1X;
  coords[1] = input->SltPt1Y;
  coords[2] = input->SltPt2X;
  coords[3] = input->SltPt2Y;
  coords[4] = input->SltPt3X;
  coords[5] = input->SltPt3Y;
  coords[6] = input->SltPt4X;
  coords[7] = input->SltPt4Y;

  if (idx0based >= 0 && idx0based < 8) {
    result0 = coords[idx0based];
  } else {
    result0 = 0.0f;
  }
  return result0;
}

/**
 * @brief 展开输入中的车位四角点，保持 MATLAB slot_pts 的 1*8 布局。
 */
/**
 * @brief 展开目标位姿，供 GroundCS_2_VehCS / calcSideLine 复用。
 */
static void checkParkingTypeFillTargetPose(const CheckParkingTypeInput_t *input, Real tarPos[3]) {
  tarPos[0] = input->TarX;
  tarPos[1] = input->TarY;
  tarPos[2] = input->TarTheta;
}

/**
 * @brief 对齐 ClassSlot.calcParkingType 的垂直车位提前返回逻辑。
 */
static Boolean checkParkingTypeVehAxleInSlot(const CheckParkingTypeInput_t *input,
                                             const Real slotPts[8]) {
  Real frontX;
  Real frontY;
  Boolean rearInSlot;
  Boolean frontInSlot;
  Boolean axleInSlot;

  rearInSlot = apaMathIsPtInRect(input->VehX, input->VehY, slotPts);
  if (rearInSlot) {
    axleInSlot = true;
  } else {
    apaVehicleFrontAxlePt(input->VehX,
                          input->VehY,
                          input->VehTheta,
                          CHECK_PARKING_TYPE_DEFAULT_WHEELBASE,
                          &frontX,
                          &frontY);
    frontInSlot = apaMathIsPtInRect(frontX, frontY, slotPts);
    axleInSlot = frontInSlot;
  }
  return axleInSlot;
}

/**
 * @brief 将当前车辆位姿表达在目标车位坐标系下。
 */
static void checkParkingTypeVehPoseInTarget(const CheckParkingTypeInput_t *input,
                                            const Real tarPos[3],
                                            Real localPose[3]) {
  Real vehPose[3];

  vehPose[0] = input->VehX;
  vehPose[1] = input->VehY;
  vehPose[2] = input->VehTheta;
  apaCoordGroundToVehPose(tarPos, vehPose, localPose);
}

/**
 * @brief 内联展开 ClassSlot.calcSlotClass
 *        比较入口边长度和侧边长度来判定车位类型。
 *        enter_edge_len > side_edge_len → PARALLEL(1)
 *        否则 → VERTICAL(0)
 *
 * @source MATLAB ClassSlot.m:392-410
 */
static Integer checkParkingTypeCalcSlotClass(const CheckParkingTypeInput_t *input) {
  Integer enterEdge0;
  Integer pt1xIdx;
  Integer pt1yIdx;
  Integer pt2xIdx;
  Integer pt2yIdx;
  Integer pt3xIdx;
  Integer pt3yIdx;
  Real pt1x;
  Real pt1y;
  Real pt2x;
  Real pt2y;
  Real pt3x;
  Real pt3y;
  Real dx12;
  Real dy12;
  Real dx23;
  Real dy23;
  Real enterEdgeLen;
  Real sideEdgeLen;
  Integer slotClass0;

  enterEdge0 = input->EnterEdge;

  pt1xIdx = checkParkingTypeWrapIdx(2 * enterEdge0 - 1);
  pt1yIdx = checkParkingTypeWrapIdx(2 * enterEdge0);
  pt2xIdx = checkParkingTypeWrapIdx(2 * enterEdge0 + 1);
  pt2yIdx = checkParkingTypeWrapIdx(2 * enterEdge0 + 2);
  pt3xIdx = checkParkingTypeWrapIdx(2 * enterEdge0 + 3);
  pt3yIdx = checkParkingTypeWrapIdx(2 * enterEdge0 + 4);

  pt1x = checkParkingTypeGetPtCoord(input, pt1xIdx);
  pt1y = checkParkingTypeGetPtCoord(input, pt1yIdx);
  pt2x = checkParkingTypeGetPtCoord(input, pt2xIdx);
  pt2y = checkParkingTypeGetPtCoord(input, pt2yIdx);
  pt3x = checkParkingTypeGetPtCoord(input, pt3xIdx);
  pt3y = checkParkingTypeGetPtCoord(input, pt3yIdx);

  dx12 = pt1x - pt2x;
  dy12 = pt1y - pt2y;
  dx23 = pt2x - pt3x;
  dy23 = pt2y - pt3y;

  enterEdgeLen = sqrtf(dx12 * dx12 + dy12 * dy12);
  sideEdgeLen = sqrtf(dx23 * dx23 + dy23 * dy23);

  if (apaMathGreater(enterEdgeLen, sideEdgeLen)) {
    slotClass0 = SLOT_CLASS_PARALLEL;
  } else {
    slotClass0 = SLOT_CLASS_VERTICAL;
  }

  return slotClass0;
}

/**
 * @brief 泊车方向推理：判定车辆相对车位的泊入方向（左/右）及车位类型（垂直/平行）
 * @en_name checkParkingType
 * @cn_name 泊车方向推理
 * @type module
 * @param[IN] Real VehX 车辆X
 * @param[IN] Real VehY 车辆Y
 * @param[IN] Real VehTheta 车辆航向
 * @param[IN] Real TarX 目标X
 * @param[IN] Real TarY 目标Y
 * @param[IN] Real TarTheta 目标航向
 * @param[IN] Real SltPt1X 车位角点1X
 * @param[IN] Real SltPt1Y 车位角点1Y
 * @param[IN] Real SltPt2X 车位角点2X
 * @param[IN] Real SltPt2Y 车位角点2Y
 * @param[IN] Real SltPt3X 车位角点3X
 * @param[IN] Real SltPt3Y 车位角点3Y
 * @param[IN] Real SltPt4X 车位角点4X
 * @param[IN] Real SltPt4Y 车位角点4Y
 * @param[IN] Integer EnterEdge 入口边编号(1-4)
 * @param[OUT] Integer ParkingType 泊车方向(0=无意义 1=左 2=右 3=未知)
 * @param[OUT] Integer SlotClass 车位类型(0=垂直 1=平行)
 * @granularity atomic
 * @tag_level1 泊车规划
 * @tag_level2 车位分析
 * @source matlab_rewrite check_parking_type.m + get_slot_class.m (内联ClassSlot)
 * @version 1.0
 * @date 2026-04-12
 * @author aide
 */
void checkParkingType(const CheckParkingTypeInput_t *input, CheckParkingTypeOutput_t *output) {
  Integer slotClass0;
  Integer parkingType0;
  Real slotPts[8];
  Real tarPos[3];

  apaSlotSetPoints(slotPts,
                   input->SltPt1X,
                   input->SltPt1Y,
                   input->SltPt2X,
                   input->SltPt2Y,
                   input->SltPt3X,
                   input->SltPt3Y,
                   input->SltPt4X,
                   input->SltPt4Y);
  checkParkingTypeFillTargetPose(input, tarPos);
  slotClass0 = checkParkingTypeCalcSlotClass(input);

  if (slotClass0 == SLOT_CLASS_VERTICAL) {

    Real localPose[3];
    Real dTheta;
    Real absDTheta;
    Real absLateral;
    Boolean condAligned;

    if (checkParkingTypeVehAxleInSlot(input, slotPts)) {
      parkingType0 = PARKING_TYPE_MEANINGLESS;
    } else {
      checkParkingTypeVehPoseInTarget(input, tarPos, localPose);

      dTheta = localPose[2];
      absLateral = fabsf(localPose[1]);
      absDTheta = fabsf(dTheta);

      condAligned = apaMathLess(absLateral, CHECK_PARKING_TYPE_LATERAL_THRESH) &&
                    apaMathLess(absDTheta, CHECK_PARKING_TYPE_YAW_THRESH);
      if (condAligned) {
        parkingType0 = PARKING_TYPE_MEANINGLESS;
      } else {
        if (apaMathLess(dTheta, 0.0f)) {
          parkingType0 = PARKING_TYPE_RIGHT;
        } else {
          parkingType0 = PARKING_TYPE_LEFT;
        }
      }
    }
  } else {
    Real lnFront[4];
    Real lnRear[4];
    Real frontPoint[3];
    Real localPoint[3];

    apaSlotCalcSideLine(slotPts, input->EnterEdge, tarPos, slotClass0, lnFront, lnRear);
    frontPoint[0] = lnFront[0];
    frontPoint[1] = lnFront[1];
    frontPoint[2] = 0.0f;
    apaCoordGroundToVehPose(tarPos, frontPoint, localPoint);

    if (apaMathGreater(localPoint[1], 0.0f)) {
      parkingType0 = PARKING_TYPE_RIGHT;
    } else {
      parkingType0 = PARKING_TYPE_LEFT;
    }
  }

  output->ParkingType = parkingType0;
  output->SlotClass = slotClass0;
}
