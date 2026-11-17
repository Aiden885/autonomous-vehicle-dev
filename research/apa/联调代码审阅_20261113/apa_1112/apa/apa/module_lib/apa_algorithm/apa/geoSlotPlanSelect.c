#include "geoSlotPlanSelectTypes.h"
#include "geoAxisTransTypes.h"
#include <math.h>

static Integer
geoSlotPlanSelectCheckDuplicate(const Integer *idArr, Integer outIdNum, Integer idTmp);
static Integer geoSlotPlanSelectAppendNoFilter(Integer inIdNum,
                                               Integer *idArr,
                                               const Integer *slotStsArr,
                                               const Integer *slotIdArr,
                                               const Integer *slotDirArr,
                                               const Real *ptYArr,
                                               const Real *ptThetaArr,
                                               Integer apaSrchSts,
                                               Real vehSpd);
static Integer geoSlotPlanSelectAppendPriority1(Integer inIdNum,
                                                Integer *idArr,
                                                const Integer *slotStsArr,
                                                const Integer *slotIdArr,
                                                const Integer *slotDirArr,
                                                const Real *ptYArr,
                                                const Real *ptThetaArr,
                                                Integer apaSrchSts,
                                                Real vehSpd,
                                                Integer filtN);

static inline void geoSlotPlanSelectPickMinList(Real *valList,
                                                const Integer *slotIdArr,
                                                Integer *idArr,
                                                Integer *outIdNumPtr) {
  Real minVal = 1000.0f;
  Integer minIdx = 0, idx;
  for (idx = 0; idx < MAX_SLOT_NUM; idx = idx + 1) {
    if (valList[idx] < minVal) {
      minVal = valList[idx];
      minIdx = idx;
    }
  }
  if (minVal < 60.0f && minVal > 1.0f) {
    idArr[*outIdNumPtr] = slotIdArr[minIdx];
    *outIdNumPtr = *outIdNumPtr + 1;
    valList[minIdx] = 100.0f;
  }
}

static Integer geoSlotPlanSelectAppendPriority2(Integer inIdNum,
                                                Integer *idArr,
                                                Real *distList,
                                                Real *distEdList,
                                                const Real *ptX,
                                                const Real *ptY,
                                                const Integer *slotStsArr,
                                                const Integer *slotSubStsArr,
                                                const Integer *slotIdArr,
                                                Integer apaSrchSts,
                                                Integer filtN);
static Integer geoSlotPlanSelectAppendPriority3(Integer inIdNum,
                                                Integer *idArr,
                                                const Integer *slotStsArr,
                                                const Integer *slotIdArr,
                                                Integer apaSrchSts,
                                                Integer filtN);
static Integer geoSlotPlanSelectAppendPriority4(Integer inIdNum,
                                                Integer *idArr,
                                                const Integer *slotStsArr,
                                                const Integer *slotIdArr,
                                                Integer apaSrchSts,
                                                Integer filtN);

/**
 * @brief 车位及规划策略分配选择器模块
 * @en_name geoSlotPlanSelect
 * @cn_name 车位分配选择器
 * @type module
 * @param[IN] GeoSlotPlanSelectInput_t Input 车位分配输入
 * @param[OUT] GeoSlotPlanSelectOutput_t Output 车位分配输出
 * @granularity composite
 * @tag_level1 泊车规划
 * @tag_level2 车位选择
 * @source matlab_rewrite geo_slotPlanSelect.m
 * @version 1.0
 * @date 2026-04-13
 * @author aide
 *
 * MATLAB 在优先级 3/4 分支用 randi 随机抽取候选车位；C 实现把 randi 替换为
 * cnt % stsN 的确定性轮询，输出顺序由桶大小和遍历顺序决定，结果可复现。
 * 调用方若依赖 MATLAB 随机分布，需要在外层重新引入随机源。
 */
void geoSlotPlanSelect(const GeoSlotPlanSelectInput_t *input, GeoSlotPlanSelectOutput_t *output) {
  Integer outIdNum = 0, kk;
  Real ptRListX[MAX_SLOT_NUM];
  Real ptRListY[MAX_SLOT_NUM];
  Real ptRListTheta[MAX_SLOT_NUM];
  Real distList[MAX_SLOT_NUM];
  Real distEdList[MAX_SLOT_NUM];
  Boolean condNoFilter;

  *output = (GeoSlotPlanSelectOutput_t){0};

  for (kk = 0; kk < MAX_SLOT_NUM; kk = kk + 1) {
    output->IdList[kk] = 0;
    ptRListX[kk] = 0.0f;
    ptRListY[kk] = 0.0f;
    ptRListTheta[kk] = 0.0f;
    distList[kk] = 100.0f;
    distEdList[kk] = 100.0f;
  }

  for (kk = 0; kk < MAX_SLOT_NUM; kk = kk + 1) {
    Boolean condValidId;
    condValidId = (input->SlotID[kk] > 0);
    if (condValidId) {
      GeoAxisTransInput_t axIn;
      GeoAxisTransOutput_t axOut;
      axIn.PtX = input->SlotTarPtX[kk];
      axIn.PtY = input->SlotTarPtY[kk];
      axIn.PtTheta = input->SlotTarPtTheta[kk];
      axIn.Ori1X = 0.0f;
      axIn.Ori1Y = 0.0f;
      axIn.Ori1Theta = 0.0f;
      axIn.Ori2X = input->VehPosX;
      axIn.Ori2Y = input->VehPosY;
      axIn.Ori2Theta = input->VehPosTheta;
      geoAxisTrans(&axIn, &axOut);

      ptRListX[kk] = axOut.ResultX;
      ptRListY[kk] = axOut.ResultY;
      ptRListTheta[kk] = axOut.ResultTheta;
    }
  }

  condNoFilter = ((input->FiltSw == 0) || (input->SlotNm <= input->FiltN));
  if (condNoFilter) {
    outIdNum = geoSlotPlanSelectAppendNoFilter(outIdNum,
                                               output->IdList,
                                               input->SlotSts,
                                               input->SlotID,
                                               input->SlotDir,
                                               ptRListY,
                                               ptRListTheta,
                                               input->ApaSrchSts,
                                               input->VehSpd);
  } else {
    outIdNum = geoSlotPlanSelectAppendPriority1(outIdNum,
                                                output->IdList,
                                                input->SlotSts,
                                                input->SlotID,
                                                input->SlotDir,
                                                ptRListY,
                                                ptRListTheta,
                                                input->ApaSrchSts,
                                                input->VehSpd,
                                                input->FiltN);
    outIdNum = geoSlotPlanSelectAppendPriority2(outIdNum,
                                                output->IdList,
                                                distList,
                                                distEdList,
                                                ptRListX,
                                                ptRListY,
                                                input->SlotSts,
                                                input->SlotSubSts,
                                                input->SlotID,
                                                input->ApaSrchSts,
                                                input->FiltN);
    outIdNum = geoSlotPlanSelectAppendPriority3(
        outIdNum, output->IdList, input->SlotSts, input->SlotID, input->ApaSrchSts, input->FiltN);
    outIdNum = geoSlotPlanSelectAppendPriority4(
        outIdNum, output->IdList, input->SlotSts, input->SlotID, input->ApaSrchSts, input->FiltN);
  }

  output->IdNum = outIdNum;
}

static Integer
geoSlotPlanSelectCheckDuplicate(const Integer *idArr, Integer outIdNum, Integer idTmp) {
  Integer kk = 0;
  Integer isDup = 0;
  while ((kk < outIdNum) && (isDup == 0)) {
    if (idTmp == idArr[kk]) {
      isDup = 1;
    }
    kk = kk + 1;
  }
  return isDup;
}

static Integer geoSlotPlanSelectAppendNoFilter(Integer inIdNum,
                                               Integer *idArr,
                                               const Integer *slotStsArr,
                                               const Integer *slotIdArr,
                                               const Integer *slotDirArr,
                                               const Real *ptYArr,
                                               const Real *ptThetaArr,
                                               Integer apaSrchSts,
                                               Real vehSpd) {
  Integer outIdNum = inIdNum;
  Integer kk;
  for (kk = 0; kk < MAX_SLOT_NUM; kk = kk + 1) {
    Boolean condValidId = (slotIdArr[kk] > 0);
    if (condValidId) {
      Integer sts = slotStsArr[kk];
      Boolean condSearching = (apaSrchSts == 1);
      if (condSearching) {
        Boolean condStsMatch = ((sts == 1) || (sts == 2) || (sts == 3) || (sts == 4));
        if (condStsMatch) {
          idArr[outIdNum] = slotIdArr[kk];
          outIdNum = outIdNum + 1;
        }
      } else {
        Boolean condParking = (apaSrchSts == 2);
        if (condParking) {
          Boolean condSubStsMatch = (sts == 6);
          Boolean condSpd = (vehSpd < 0.2f);
          Boolean condPt = ((fabsf(ptYArr[kk]) > 0.5f) ||
                            (fabsf(ptThetaArr[kk]) > (APA_PI / 10.0f)) || (slotDirArr[kk] != 1));
          if (condSubStsMatch) {
            if (condSpd) {
              if (condPt) {
                idArr[outIdNum] = slotIdArr[kk];
                outIdNum = outIdNum + 1;
              }
            }
          }
        }
      }
    }
  }
  return outIdNum;
}

static Integer geoSlotPlanSelectAppendPriority1(Integer inIdNum,
                                                Integer *idArr,
                                                const Integer *slotStsArr,
                                                const Integer *slotIdArr,
                                                const Integer *slotDirArr,
                                                const Real *ptYArr,
                                                const Real *ptThetaArr,
                                                Integer apaSrchSts,
                                                Real vehSpd,
                                                Integer filtN) {
  Integer outIdNum = inIdNum;
  Integer kk;
  for (kk = 0; kk < MAX_SLOT_NUM; kk = kk + 1) {
    Boolean condValidId = (slotIdArr[kk] > 0);
    Boolean condNotFull = (outIdNum < filtN);
    if (condValidId) {
      if (condNotFull) {
        Integer sts = slotStsArr[kk];
        Boolean condSearching = (apaSrchSts == 1);
        if (condSearching) {
          if (sts == 1) {
            idArr[outIdNum] = slotIdArr[kk];
            outIdNum = outIdNum + 1;
          }
        } else {
          Boolean condParking = (apaSrchSts == 2);
          if (condParking) {
            if (sts == 6) {
              Boolean condSpd = (vehSpd < 0.2f);
              Boolean condPt =
                  ((fabsf(ptYArr[kk]) > 0.5f) || (fabsf(ptThetaArr[kk]) > (APA_PI / 10.0f)) ||
                   (slotDirArr[kk] != 1));
              if (condSpd && condPt) {
                idArr[outIdNum] = slotIdArr[kk];
                outIdNum = outIdNum + 1;
              }
            }
          }
        }
      }
    }
  }
  return outIdNum;
}

static Integer geoSlotPlanSelectAppendPriority2(Integer inIdNum,
                                                Integer *idArr,
                                                Real *distList,
                                                Real *distEdList,
                                                const Real *ptX,
                                                const Real *ptY,
                                                const Integer *slotStsArr,
                                                const Integer *slotSubStsArr,
                                                const Integer *slotIdArr,
                                                Integer apaSrchSts,
                                                Integer filtN) {
  Integer outIdNum = inIdNum;
  Boolean condPri2 = ((apaSrchSts == 1) && (outIdNum < filtN));
  if (condPri2) {
    Integer sts2N = 0, sts2Ned = 0, kk;
    for (kk = 0; kk < MAX_SLOT_NUM; kk = kk + 1) {
      Boolean condId2 = ((slotIdArr[kk] > 0) && (slotStsArr[kk] == 2));
      if (condId2) {
        Boolean condSub0 = (slotSubStsArr[kk] > 0);
        if (condSub0) {
          sts2Ned = sts2Ned + 1;
          distEdList[kk] = sqrtf(ptX[kk] * ptX[kk] + ptY[kk] * ptY[kk]);
        } else {
          sts2N = sts2N + 1;
          distList[kk] = sqrtf(ptX[kk] * ptX[kk] + ptY[kk] * ptY[kk]);
        }
      }
    }
    Boolean condSts2Loop = ((sts2N > 0) && (outIdNum < filtN));
    while (condSts2Loop) {
      geoSlotPlanSelectPickMinList(distList, slotIdArr, idArr, &outIdNum);
      sts2N = sts2N - 1;
      condSts2Loop = ((sts2N > 0) && (outIdNum < filtN));
    }
    Boolean condSts2EdLoop = ((sts2Ned > 0) && (outIdNum < filtN));
    while (condSts2EdLoop) {
      geoSlotPlanSelectPickMinList(distEdList, slotIdArr, idArr, &outIdNum);
      sts2Ned = sts2Ned - 1;
      condSts2EdLoop = ((sts2Ned > 0) && (outIdNum < filtN));
    }
  }
  return outIdNum;
}

static Integer geoSlotPlanSelectAppendPriority3(Integer inIdNum,
                                                Integer *idArr,
                                                const Integer *slotStsArr,
                                                const Integer *slotIdArr,
                                                Integer apaSrchSts,
                                                Integer filtN) {
  Integer outIdNum = inIdNum;
  Boolean condPri3 = ((apaSrchSts == 1) && (outIdNum < filtN));
  if (condPri3) {
    Integer sts4N = 0, idx, kk;
    Integer kklist[MAX_SLOT_NUM];
    for (idx = 0; idx < MAX_SLOT_NUM; idx = idx + 1) {
      kklist[idx] = 0;
    }
    for (kk = 0; kk < MAX_SLOT_NUM; kk = kk + 1) {
      Boolean condId4 = ((slotIdArr[kk] > 0) && (slotStsArr[kk] == 4));
      if (condId4) {
        kklist[sts4N] = kk;
        sts4N = sts4N + 1;
      }
    }
    Boolean condSts4Loop;
    Integer cnt = 0;
    condSts4Loop = ((sts4N > 0) && (outIdNum < filtN) && (cnt < 20));
    while (condSts4Loop) {

      Integer pckIdx = cnt % sts4N;
      Integer idTmp = slotIdArr[kklist[pckIdx]];
      Integer idxSamFlg = geoSlotPlanSelectCheckDuplicate(idArr, outIdNum, idTmp);
      if (idxSamFlg == 0) {
        idArr[outIdNum] = idTmp;
        outIdNum = outIdNum + 1;
      }
      cnt = cnt + 1;
      condSts4Loop = ((sts4N > 0) && (outIdNum < filtN) && (cnt < 20));
    }
  }
  return outIdNum;
}

static Integer geoSlotPlanSelectAppendPriority4(Integer inIdNum,
                                                Integer *idArr,
                                                const Integer *slotStsArr,
                                                const Integer *slotIdArr,
                                                Integer apaSrchSts,
                                                Integer filtN) {
  Integer outIdNum = inIdNum;
  Boolean condPri4 = ((apaSrchSts == 1) && (outIdNum < filtN));
  if (condPri4) {
    Integer sts3N = 0, idx, kk;
    Integer kklist[MAX_SLOT_NUM];
    for (idx = 0; idx < MAX_SLOT_NUM; idx = idx + 1) {
      kklist[idx] = 0;
    }
    for (kk = 0; kk < MAX_SLOT_NUM; kk = kk + 1) {
      Boolean condId3 = ((slotIdArr[kk] > 0) && (slotStsArr[kk] == 3));
      if (condId3) {
        kklist[sts3N] = kk;
        sts3N = sts3N + 1;
      }
    }
    Boolean condSts3Loop;
    Integer cnt = 0;
    condSts3Loop = ((sts3N > 0) && (outIdNum < filtN) && (cnt < 20));
    while (condSts3Loop) {

      Integer pckIdx = cnt % sts3N;
      Integer idTmp = slotIdArr[kklist[pckIdx]];
      Integer idxSamFlg = geoSlotPlanSelectCheckDuplicate(idArr, outIdNum, idTmp);
      if (idxSamFlg == 0) {
        idArr[outIdNum] = idTmp;
        outIdNum = outIdNum + 1;
      }
      cnt = cnt + 1;
      condSts3Loop = ((sts3N > 0) && (outIdNum < filtN) && (cnt < 20));
    }
  }
  return outIdNum;
}
