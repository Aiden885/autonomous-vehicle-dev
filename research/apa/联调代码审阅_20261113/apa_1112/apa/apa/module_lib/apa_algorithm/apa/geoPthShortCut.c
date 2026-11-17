#include "geoPthShortCutTypes.h"
#include <math.h>

static void geoPthShortCutBacktrack(const GeoPthShortCutInput_t *input,
                                    const GeoPthShortCutOutput_t *output,
                                    Integer startIdx,
                                    Real cutXrx,
                                    Real cosVeh,
                                    Real sinVeh,
                                    Integer *outBackIdx);
static void geoPthShortCutApplyCut(const GeoPthShortCutInput_t *input,
                                   GeoPthShortCutOutput_t *output,
                                   Integer cutIdx1,
                                   Integer cutIdx2,
                                   Integer *ptCount);
static Boolean geoPthShortCutCheckEnable(const GeoPthShortCutInput_t *input, Integer ptCount);

static Boolean geoPthShortCutCheckEnable(const GeoPthShortCutInput_t *input, Integer ptCount) {
  Boolean condEnable = (input->ShortCutSw == 1);
  Boolean condSegNum = (input->SegNum > 1);
  Boolean condSegType = (input->Seg1TrnDir == 0) && (input->Seg2TrnDir != 0);
  Boolean condGoDir = (input->Seg1GoDir != input->Seg2GoDir);
  Boolean condSegLen = (input->Seg2Len > 2.0f);
  Boolean condStp2 = (input->Stp2Idx > 0);
  Boolean condPtCount = (ptCount > 0);
  return condEnable && condSegNum && condSegType && condGoDir && condSegLen && condStp2 &&
         condPtCount;
}

static void geoPthShortCutBacktrack(const GeoPthShortCutInput_t *input,
                                    const GeoPthShortCutOutput_t *output,
                                    Integer startIdx,
                                    Real cutXrx,
                                    Real cosVeh,
                                    Real sinVeh,
                                    Integer *outBackIdx) {
  Integer backIdx = startIdx;
  Boolean backDone = false;

  while ((backIdx >= 0) && (!backDone)) {
    Real bx = output->PthPtX[backIdx];
    Real by = output->PthPtY[backIdx];
    Real bdX = bx - input->VehPosX;
    Real bdY = by - input->VehPosY;
    Real brX = cosVeh * bdX + sinVeh * bdY;
    Real mulA = brX;
    Real mulB = brX - cutXrx;

    Boolean condCross = ((mulA * mulB) < 0.0f);
    if (condCross) {
      Integer nextIdx = backIdx + 1;
      if (nextIdx > startIdx) {
        backIdx = startIdx;
      } else {
        backIdx = nextIdx;
      }
      backDone = true;
    } else {
      backIdx = backIdx - 1;
    }
  }

  if (backIdx < 0) {
    backIdx = 0;
  }
  *outBackIdx = backIdx;
}

static void geoPthShortCutApplyCut(const GeoPthShortCutInput_t *input,
                                   GeoPthShortCutOutput_t *output,
                                   Integer cutIdx1,
                                   Integer cutIdx2,
                                   Integer *ptCount) {
  Integer deleteCount = cutIdx2 - cutIdx1 - 1;
  Integer dstIdx = cutIdx1 + 1;

  while (dstIdx < (*ptCount - deleteCount)) {
    Integer srcIdx = dstIdx + deleteCount;
    output->PthPtX[dstIdx] = output->PthPtX[srcIdx];
    output->PthPtY[dstIdx] = output->PthPtY[srcIdx];
    output->PthPtTht[dstIdx] = output->PthPtTht[srcIdx];
    output->PthPtOdo[dstIdx] = output->PthPtOdo[srcIdx];
    dstIdx = dstIdx + 1;
  }

  Integer zeroIdx = *ptCount - deleteCount;
  while (zeroIdx < *ptCount) {
    output->PthPtX[zeroIdx] = 0.0f;
    output->PthPtY[zeroIdx] = 0.0f;
    output->PthPtTht[zeroIdx] = 0.0f;
    output->PthPtOdo[zeroIdx] = 0.0f;
    zeroIdx = zeroIdx + 1;
  }

  *ptCount = *ptCount - deleteCount;
  output->PtCount = *ptCount;
  output->CutApplied = 1;

  if ((cutIdx1 >= 0) && ((cutIdx1 + 1) < *ptCount)) {
    Real dX = output->PthPtX[cutIdx1] - output->PthPtX[cutIdx1 + 1];
    Real dY = output->PthPtY[cutIdx1] - output->PthPtY[cutIdx1 + 1];
    Real segLen = sqrtf(dX * dX + dY * dY);

    if (input->Seg1GoDir > 0) {
      output->PthPtOdo[cutIdx1] = -segLen;
    } else {
      output->PthPtOdo[cutIdx1] = segLen;
    }
  }
}

/**
 * @brief 对第一段直行与第二段转弯换向场景执行轨迹shortcut，删除偏离阈值范围内的冗余点
 * @en_name geoPthShortCut
 * @cn_name 轨迹shortcut
 * @type module
 * @param[IN] Integer ShortCutSw 是否启用shortcut
 * @param[IN] Integer SegNum 路径段数
 * @param[IN] Integer Seg1TrnDir 第1段转向
 * @param[IN] Integer Seg2TrnDir 第2段转向
 * @param[IN] Integer Seg1GoDir 第1段行驶方向
 * @param[IN] Integer Seg2GoDir 第2段行驶方向
 * @param[IN] Real Seg1Len 第1段长度
 * @param[IN] Real Seg2Len 第2段长度
 * @param[IN] Integer Stp2Idx 第2段起始索引
 * @param[IN] Integer PtCountIn 输入点数
 * @param[IN] Real[100] PthPtXIn 输入轨迹X
 * @param[IN] Real[100] PthPtYIn 输入轨迹Y
 * @param[IN] Real[100] PthPtThtIn 输入轨迹航向
 * @param[IN] Real[100] PthPtOdoIn 输入轨迹里程
 * @param[IN] Real VehPosX 车辆X
 * @param[IN] Real VehPosY 车辆Y
 * @param[IN] Real VehPosTheta 车辆航向
 * @param[IN] Real ShortCutThd shortcut横向阈值
 * @param[OUT] Integer PtCount 输出点数
 * @param[OUT] Integer CutApplied 是否执行cut
 * @param[OUT] Real[100] PthPtX 输出轨迹X
 * @param[OUT] Real[100] PthPtY 输出轨迹Y
 * @param[OUT] Real[100] PthPtTht 输出轨迹航向
 * @param[OUT] Real[100] PthPtOdo 输出轨迹里程
 * @granularity atomic
 * @tag_level1 泊车规划
 * @tag_level2 路径离散化
 * @source matlab_rewrite geo_PthShortCut.m
 * @version 1.0
 * @date 2026-03-04
 * @author aide
 */
void geoPthShortCut(const GeoPthShortCutInput_t *input, GeoPthShortCutOutput_t *output) {
  Integer ptCount;
  Integer idx;
  Real cosVeh;
  Real sinVeh;

  ptCount = input->PtCountIn;
  output->PtCount = ptCount;
  output->CutApplied = 0;

  idx = 0;
  while (idx < PATH_PT_MAX) {
    output->PthPtX[idx] = input->PthPtXIn[idx];
    output->PthPtY[idx] = input->PthPtYIn[idx];
    output->PthPtTht[idx] = input->PthPtThtIn[idx];
    output->PthPtOdo[idx] = input->PthPtOdoIn[idx];
    idx = idx + 1;
  }

  cosVeh = cosf(input->VehPosTheta);
  sinVeh = sinf(input->VehPosTheta);

  Boolean condNeedCut = geoPthShortCutCheckEnable(input, ptCount);

  if (condNeedCut) {
    Integer startIdx;
    Integer searchIdx;
    Integer cutIdx1;
    Integer cutIdx2;
    Integer act;
    Real cutXrx;

    startIdx = input->Stp2Idx - 1;
    if (startIdx < 0) {
      startIdx = 0;
    }
    if (startIdx >= ptCount) {
      startIdx = ptCount - 1;
    }

    searchIdx = startIdx;
    cutIdx1 = 0;
    cutIdx2 = 0;
    act = 0;

    while (searchIdx < ptCount) {
      Real ptX;
      Real ptY;
      Real dX;
      Real dY;
      Real ptRx;
      Real ptRy;
      Real absRy;

      ptX = output->PthPtX[searchIdx];
      ptY = output->PthPtY[searchIdx];
      dX = ptX - input->VehPosX;
      dY = ptY - input->VehPosY;
      ptRx = cosVeh * dX + sinVeh * dY;
      ptRy = cosVeh * dY - sinVeh * dX;
      absRy = fabsf(ptRy);

      Boolean condOut;
      condOut = (absRy > input->ShortCutThd);
      if (condOut) {
        Integer backIdx;

        cutIdx2 = searchIdx - 1;
        if (cutIdx2 < 0) {
          cutIdx2 = 0;
        }
        cutXrx = ptRx;

        geoPthShortCutBacktrack(input, output, startIdx, cutXrx, cosVeh, sinVeh, &backIdx);

        cutIdx1 = backIdx;
        if (cutIdx2 > (cutIdx1 + 1)) {
          act = 1;
        }
        searchIdx = ptCount;
      } else {
        searchIdx = searchIdx + 1;
      }
    }

    if (act == 1) {
      geoPthShortCutApplyCut(input, output, cutIdx1, cutIdx2, &ptCount);
    }
  }
}
