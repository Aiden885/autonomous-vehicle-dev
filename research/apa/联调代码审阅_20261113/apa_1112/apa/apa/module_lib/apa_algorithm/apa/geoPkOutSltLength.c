#include "apaCountUtils.h"
#include "apaMathUtils.h"
#include "geoPkOutSltLengthTypes.h"
#include <math.h>

static inline Real geoPkOutSltLengthCalcLen(Integer pkOutDir, const Real *slotPtR) {
  Real len = 0.0f;
  if (pkOutDir == 3) {
    if ((slotPtR[0] > 0.0f) && (slotPtR[2] > 0.0f)) {
      len = apaMathMax(slotPtR[0], slotPtR[2]);
    } else if ((slotPtR[2] > 0.0f) && (slotPtR[4] > 0.0f)) {
      len = apaMathMax(slotPtR[2], slotPtR[4]);
    } else if ((slotPtR[4] > 0.0f) && (slotPtR[6] > 0.0f)) {
      len = apaMathMax(slotPtR[4], slotPtR[6]);
    } else if ((slotPtR[0] > 0.0f) && (slotPtR[6] > 0.0f)) {
      len = apaMathMax(slotPtR[0], slotPtR[6]);
    }
  } else if (pkOutDir == 4) {
    if ((slotPtR[0] < 0.0f) && (slotPtR[2] < 0.0f)) {
      len = apaMathMax(-slotPtR[0], -slotPtR[2]);
    } else if ((slotPtR[2] < 0.0f) && (slotPtR[4] < 0.0f)) {
      len = apaMathMax(-slotPtR[2], -slotPtR[4]);
    } else if ((slotPtR[4] < 0.0f) && (slotPtR[6] < 0.0f)) {
      len = apaMathMax(-slotPtR[4], -slotPtR[6]);
    } else if ((slotPtR[0] < 0.0f) && (slotPtR[6] < 0.0f)) {
      len = apaMathMax(-slotPtR[0], -slotPtR[6]);
    }
  }
  return len;
}

static Real geoPkOutSltLengthLineValue(Real ptX, Real ptY, Real x0, Real y0, Real x1, Real y1) {
  Real lineA;
  Real lineB;
  Real lineC;
  Real value;

  lineA = y1 - y0;
  lineB = x0 - x1;
  lineC = (x1 * y0) - (x0 * y1);
  value = (ptX * lineA) + (ptY * lineB);
  value = value + lineC;
  return value;
}

static Boolean geoPkOutSltLengthIsInBoxStrict(Real ptX, Real ptY, const Real rect[8]) {
  Real val12;
  Real val43;
  Real val14;
  Real val23;
  Boolean inside;

  val12 = geoPkOutSltLengthLineValue(ptX, ptY, rect[0], rect[1], rect[2], rect[3]);
  val43 = geoPkOutSltLengthLineValue(ptX, ptY, rect[6], rect[7], rect[4], rect[5]);
  val14 = geoPkOutSltLengthLineValue(ptX, ptY, rect[0], rect[1], rect[6], rect[7]);
  val23 = geoPkOutSltLengthLineValue(ptX, ptY, rect[2], rect[3], rect[4], rect[5]);

  inside = false;
  if (((val12 * val43) < 0.0f) && ((val14 * val23) < 0.0f)) {
    inside = true;
  }
  return inside;
}

/**
 * @file geoPkOutSltLength.c
 * @brief 泊出槽位有效预留长度计算
 * @en_name geoPkOutSltLength
 * @cn_name 泊出槽位有效预留长度计算
 * @type module
 * @param[IN] GeoPkOutSltLengthInput_t Input 泊出槽位输入
 * @param[OUT] GeoPkOutSltLengthOutput_t Output 泊出槽位长度输出
 * @granularity composite
 * @tag_level1 泊车规划
 * @tag_level2 几何计算
 * @source matlab_rewrite geo_PkOut_SltLength.m
 * @version 1.0
 * @date 2026-04-13
 * @author aide
 */
void geoPkOutSltLength(const GeoPkOutSltLengthInput_t *input, GeoPkOutSltLengthOutput_t *output) {
  Real pkOutSltLength = 0.0f;
  Boolean inbxFlg = false;
  Integer ii;
  Integer foundIdx = -1;
  Boolean loopCond;
  Real rect[8];
  Real cosVeh;
  Real sinVeh;
  Real slotPtR[8];
  Integer p;
  Integer slotCount;
  Real dx;
  Real dy;

  if (input->InSlot) {
    slotCount = apaClampCount(input->SlotNum, 10);
    if (slotCount > 0) {
      loopCond = true;
      ii = 0;
      while (loopCond) {
        if (ii >= slotCount) {
          loopCond = false;
        }
        if (loopCond) {
          rect[0] = input->SlotPtsX[ii][0];
          rect[1] = input->SlotPtsY[ii][0];
          rect[2] = input->SlotPtsX[ii][1];
          rect[3] = input->SlotPtsY[ii][1];
          rect[4] = input->SlotPtsX[ii][2];
          rect[5] = input->SlotPtsY[ii][2];
          rect[6] = input->SlotPtsX[ii][3];
          rect[7] = input->SlotPtsY[ii][3];

          if (geoPkOutSltLengthIsInBoxStrict(input->VehPosX, input->VehPosY, rect)) {
            inbxFlg = true;
            foundIdx = ii;
            loopCond = false;
          }
        }
        if (loopCond) {
          ii = ii + 1;
        }
      }

      if (inbxFlg && (foundIdx >= 0) && (foundIdx < 10)) {
        cosVeh = cosf(input->VehPosTheta);
        sinVeh = sinf(input->VehPosTheta);

        for (p = 0; p < 4; p = p + 1) {
          dx = input->SlotPtsX[foundIdx][p] - input->VehPosX;
          dy = input->SlotPtsY[foundIdx][p] - input->VehPosY;
          slotPtR[p * 2] = dx * cosVeh + dy * sinVeh;
          slotPtR[p * 2 + 1] = -dx * sinVeh + dy * cosVeh;
        }

        pkOutSltLength = geoPkOutSltLengthCalcLen(input->PkOutDir, slotPtR);
      }
    }
  }

  output->PkOutSltLength = pkOutSltLength;
}
