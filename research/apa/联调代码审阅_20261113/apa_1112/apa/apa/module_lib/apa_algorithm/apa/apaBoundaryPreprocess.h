/**
 * @file apaBoundaryPreprocess.h
 * @brief 第三边界生成器使用的边界预处理 helper
 *
 * 对应 MATLAB:
 *   - boundary_process.m
 *   - boundary_cluster_process.m
 *   - boundary_cluster_detected_process.m
 *   - boundary_cluster_undetected_process.m
 *   - boundary_pts_detected_process.m
 *   - get_1st_boundary.m
 *
 * 这些 helper 是独立的边界点列预处理工具：把"含 (0,0) 虚拟分隔"
 * 或"含 class 标记"的点列拆为 (Boundary_Line[N][4], Boundary_Separate_Pts[M][2])，
 * 中间用 collinear_detection 简化共线段。
 *
 * DEV_RULES 合规：单出口、无 early return、变量在块顶、无 ?:、
 * 复合条件预提取、for 用 i = i + 1。
 *
 * @date 2026-05-02
 */
#ifndef APA_BOUNDARY_PREPROCESS_H
#define APA_BOUNDARY_PREPROCESS_H

#include "apaEnums.h"
#include "apaGeoUtils.h"
#include "apaMathUtils.h"
#include "baseTypes.h"

/** @brief 预处理 helper 内部点缓冲最大容量（与 MATLAB 的 520 对齐） */
#ifndef APA_BOUNDARY_PREPROC_MAX_PTS
#define APA_BOUNDARY_PREPROC_MAX_PTS 520
#endif

/** @brief 预处理 helper 内部线段缓冲最大容量 */
#ifndef APA_BOUNDARY_PREPROC_MAX_LINES
#define APA_BOUNDARY_PREPROC_MAX_LINES 520
#endif

/** @brief 预处理 helper 内部 collinear 简化最大点数（受 apaGeoCollinearDetection 限制） */
#ifndef APA_BOUNDARY_PREPROC_MAX_COLLINEAR
#define APA_BOUNDARY_PREPROC_MAX_COLLINEAR 256
#endif

/** @brief boundary_pts_class==2 表示该点是虚拟分隔值，对齐 MATLAB */
#define APA_BOUNDARY_CLASS_VIRTUAL 2

/** @brief 共线检测阈值（MATLAB 默认 0.01m） */
#ifndef APA_BOUNDARY_PREPROC_COLLINEAR_THRESHOLD
#define APA_BOUNDARY_PREPROC_COLLINEAR_THRESHOLD 0.01f
#endif

/** @brief 边界预处理输出结构 */
typedef struct {
  Real BoundaryLine[APA_BOUNDARY_PREPROC_MAX_LINES][4];
  Integer BoundaryLineN;
  Real BoundarySeparatePts[APA_BOUNDARY_PREPROC_MAX_PTS][2];
  Integer BoundarySeparatePtsN;
} ApaBoundaryPreprocOutput_t;

static inline Boolean apaBoundaryIsVirtualPoint(Real x, Real y) {
  Boolean isXZero;
  Boolean isYZero;
  Boolean isVirtual;
  isXZero = apaMathEqual(x, 0.0f);
  isYZero = apaMathEqual(y, 0.0f);
  isVirtual = isXZero && isYZero;
  return isVirtual;
}

static inline void apaBoundaryFlushTempBuffer(const Real tempPts[][2],
                                              Integer tempCount,
                                              ApaBoundaryPreprocOutput_t *output) {
  if (tempCount == 1) {
    if (output->BoundarySeparatePtsN < APA_BOUNDARY_PREPROC_MAX_PTS) {
      output->BoundarySeparatePts[output->BoundarySeparatePtsN][0] = tempPts[0][0];
      output->BoundarySeparatePts[output->BoundarySeparatePtsN][1] = tempPts[0][1];
      output->BoundarySeparatePtsN = output->BoundarySeparatePtsN + 1;
    }
  } else if (tempCount > 1) {
    Real simpPts[APA_BOUNDARY_PREPROC_MAX_COLLINEAR][2];
    Integer simpCount;
    Boolean isOk;
    Real workPts[APA_BOUNDARY_PREPROC_MAX_COLLINEAR][2];
    Integer workCount;
    Integer i;

    workCount = tempCount;
    if (workCount > APA_BOUNDARY_PREPROC_MAX_COLLINEAR) {
      workCount = APA_BOUNDARY_PREPROC_MAX_COLLINEAR;
    }
    for (i = 0; i < workCount; i = i + 1) {
      workPts[i][0] = tempPts[i][0];
      workPts[i][1] = tempPts[i][1];
    }
    isOk = apaGeoCollinearDetection((const Real(*)[2])workPts,
                                    workCount,
                                    APA_BOUNDARY_PREPROC_COLLINEAR_THRESHOLD,
                                    simpPts,
                                    &simpCount);
    if (!isOk) {
      simpCount = workCount;
      for (i = 0; i < workCount; i = i + 1) {
        simpPts[i][0] = workPts[i][0];
        simpPts[i][1] = workPts[i][1];
      }
    }
    for (i = 0; i < (simpCount - 1); i = i + 1) {
      if (output->BoundaryLineN < APA_BOUNDARY_PREPROC_MAX_LINES) {
        output->BoundaryLine[output->BoundaryLineN][0] = simpPts[i][0];
        output->BoundaryLine[output->BoundaryLineN][1] = simpPts[i][1];
        output->BoundaryLine[output->BoundaryLineN][2] = simpPts[i + 1][0];
        output->BoundaryLine[output->BoundaryLineN][3] = simpPts[i + 1][1];
        output->BoundaryLineN = output->BoundaryLineN + 1;
      }
    }
  }
}

static inline void apaBoundaryPreprocOutputInit(ApaBoundaryPreprocOutput_t *output) {
  Integer i;
  output->BoundaryLineN = 0;
  output->BoundarySeparatePtsN = 0;
  for (i = 0; i < APA_BOUNDARY_PREPROC_MAX_LINES; i = i + 1) {
    output->BoundaryLine[i][0] = 0.0f;
    output->BoundaryLine[i][1] = 0.0f;
    output->BoundaryLine[i][2] = 0.0f;
    output->BoundaryLine[i][3] = 0.0f;
  }
  for (i = 0; i < APA_BOUNDARY_PREPROC_MAX_PTS; i = i + 1) {
    output->BoundarySeparatePts[i][0] = 0.0f;
    output->BoundarySeparatePts[i][1] = 0.0f;
  }
}

/**
 * @brief boundary_process.m — 含 class 数组的边界点列预处理
 *
 * 行为：当 N<4 直接返回；遍历每点，class!=2 时入临时缓冲；遇到 class==2
 * 或末点时：临时长度==1 且 (i==2 && prev class!=2) 或 (i==N && pts[1] class!=2)
 * 时丢弃孤立点；否则刷出 separate(==1) 或 line(>1)。
 */
static inline void apaBoundaryProcess(const Real boundaryPts[][2],
                                      const UInteger8 *boundaryPtsClass,
                                      Integer boundaryPtsN,
                                      ApaBoundaryPreprocOutput_t *output) {
  Integer i;
  Real tempPts[APA_BOUNDARY_PREPROC_MAX_COLLINEAR][2];
  Integer tempCount;

  apaBoundaryPreprocOutputInit(output);
  if (boundaryPtsN < 4) {
    return;
  }
  tempCount = 0;
  for (i = 0; i < boundaryPtsN; i = i + 1) {
    Boolean isVirtual;
    Boolean isLastIdx;
    Boolean flushTemp;

    isVirtual = (boundaryPtsClass[i] == APA_BOUNDARY_CLASS_VIRTUAL);
    isLastIdx = (i == (boundaryPtsN - 1));
    flushTemp = isVirtual;
    if (isLastIdx) {
      flushTemp = true;
    }
    if (!isVirtual) {
      if (tempCount < APA_BOUNDARY_PREPROC_MAX_COLLINEAR) {
        tempPts[tempCount][0] = boundaryPts[i][0];
        tempPts[tempCount][1] = boundaryPts[i][1];
        tempCount = tempCount + 1;
      }
    }

    if (flushTemp) {

      if (tempCount == 1) {
        Boolean dropAtStart;
        Boolean dropAtEnd;
        dropAtStart =
            ((i == 1) && (boundaryPtsClass[boundaryPtsN - 2] != APA_BOUNDARY_CLASS_VIRTUAL));
        dropAtEnd = (isLastIdx && (boundaryPtsClass[1] != APA_BOUNDARY_CLASS_VIRTUAL));
        if (!dropAtStart && !dropAtEnd) {
          if (output->BoundarySeparatePtsN < APA_BOUNDARY_PREPROC_MAX_PTS) {
            output->BoundarySeparatePts[output->BoundarySeparatePtsN][0] = tempPts[0][0];
            output->BoundarySeparatePts[output->BoundarySeparatePtsN][1] = tempPts[0][1];
            output->BoundarySeparatePtsN = output->BoundarySeparatePtsN + 1;
          }
        }
      } else if (tempCount > 1) {
        apaBoundaryFlushTempBuffer((const Real(*)[2])tempPts, tempCount, output);
      }
      tempCount = 0;
    }
  }
}

/**
 * @brief boundary_cluster_process.m — cluster 边界点列预处理（带前 4 个 separate）
 *
 * 行为：前 4 行非零点直接当 separate；从第 5 行开始按 (0,0) 分割 cluster；
 * 连续 2 个 (0,0) 时提前 return。
 */
static inline void apaBoundaryClusterProcess(const Real boundaryPts[][2],
                                             Integer boundaryPtsTotal,
                                             ApaBoundaryPreprocOutput_t *output) {
  Integer i;
  Real tempPts[APA_BOUNDARY_PREPROC_MAX_COLLINEAR][2];
  Integer tempCount;
  UInteger8 virtualCnt;
  Boolean stopProcessing;

  apaBoundaryPreprocOutputInit(output);

  for (i = 0; (i < 4) && (i < boundaryPtsTotal); i = i + 1) {
    Boolean isVirtual;
    isVirtual = apaBoundaryIsVirtualPoint(boundaryPts[i][0], boundaryPts[i][1]);
    if (!isVirtual) {
      if (output->BoundarySeparatePtsN < APA_BOUNDARY_PREPROC_MAX_PTS) {
        output->BoundarySeparatePts[output->BoundarySeparatePtsN][0] = boundaryPts[i][0];
        output->BoundarySeparatePts[output->BoundarySeparatePtsN][1] = boundaryPts[i][1];
        output->BoundarySeparatePtsN = output->BoundarySeparatePtsN + 1;
      }
    }
  }

  tempCount = 0;
  virtualCnt = 0;
  stopProcessing = false;
  for (i = 4; (i < boundaryPtsTotal) && !stopProcessing; i = i + 1) {
    Boolean isVirtual;
    isVirtual = apaBoundaryIsVirtualPoint(boundaryPts[i][0], boundaryPts[i][1]);
    if (!isVirtual) {
      if (tempCount < APA_BOUNDARY_PREPROC_MAX_COLLINEAR) {
        tempPts[tempCount][0] = boundaryPts[i][0];
        tempPts[tempCount][1] = boundaryPts[i][1];
        tempCount = tempCount + 1;
      }
      virtualCnt = 0;
    } else {
      virtualCnt = virtualCnt + 1;
      if (virtualCnt > 1) {
        stopProcessing = true;
      } else {
        apaBoundaryFlushTempBuffer((const Real(*)[2])tempPts, tempCount, output);
        tempCount = 0;
      }
    }
  }
}

/**
 * @brief boundary_cluster_detected_process.m — cluster (detected) 预处理
 *
 * 与 apaBoundaryClusterProcess 一致，但不在连续 2 个 (0,0) 时提前 return
 * (MATLAB 源码该分支被注释)。
 */
static inline void apaBoundaryClusterDetectedProcess(const Real boundaryPts[][2],
                                                     Integer boundaryPtsTotal,
                                                     ApaBoundaryPreprocOutput_t *output) {
  Integer i;
  Real tempPts[APA_BOUNDARY_PREPROC_MAX_COLLINEAR][2];
  Integer tempCount;

  apaBoundaryPreprocOutputInit(output);

  for (i = 0; (i < 4) && (i < boundaryPtsTotal); i = i + 1) {
    Boolean isVirtual;
    isVirtual = apaBoundaryIsVirtualPoint(boundaryPts[i][0], boundaryPts[i][1]);
    if (!isVirtual) {
      if (output->BoundarySeparatePtsN < APA_BOUNDARY_PREPROC_MAX_PTS) {
        output->BoundarySeparatePts[output->BoundarySeparatePtsN][0] = boundaryPts[i][0];
        output->BoundarySeparatePts[output->BoundarySeparatePtsN][1] = boundaryPts[i][1];
        output->BoundarySeparatePtsN = output->BoundarySeparatePtsN + 1;
      }
    }
  }

  tempCount = 0;
  for (i = 4; i < boundaryPtsTotal; i = i + 1) {
    Boolean isVirtual;
    isVirtual = apaBoundaryIsVirtualPoint(boundaryPts[i][0], boundaryPts[i][1]);
    if (!isVirtual) {
      if (tempCount < APA_BOUNDARY_PREPROC_MAX_COLLINEAR) {
        tempPts[tempCount][0] = boundaryPts[i][0];
        tempPts[tempCount][1] = boundaryPts[i][1];
        tempCount = tempCount + 1;
      }
    } else {
      apaBoundaryFlushTempBuffer((const Real(*)[2])tempPts, tempCount, output);
      tempCount = 0;
    }
  }
}

/**
 * @brief boundary_cluster_undetected_process.m — cluster (undetected) 预处理
 *
 * 行为：从第 1 个点开始累积（无前 4 个 separate 段），按 (0,0) 分割。
 */
static inline void apaBoundaryClusterUndetectedProcess(const Real boundaryPts[][2],
                                                       Integer boundaryPtsTotal,
                                                       ApaBoundaryPreprocOutput_t *output) {
  Integer i;
  Real tempPts[APA_BOUNDARY_PREPROC_MAX_COLLINEAR][2];
  Integer tempCount;

  apaBoundaryPreprocOutputInit(output);

  tempCount = 0;
  for (i = 0; i < boundaryPtsTotal; i = i + 1) {
    Boolean isVirtual;
    isVirtual = apaBoundaryIsVirtualPoint(boundaryPts[i][0], boundaryPts[i][1]);
    if (!isVirtual) {
      if (tempCount < APA_BOUNDARY_PREPROC_MAX_COLLINEAR) {
        tempPts[tempCount][0] = boundaryPts[i][0];
        tempPts[tempCount][1] = boundaryPts[i][1];
        tempCount = tempCount + 1;
      }
    } else {
      apaBoundaryFlushTempBuffer((const Real(*)[2])tempPts, tempCount, output);
      tempCount = 0;
    }
  }
}

/**
 * @brief boundary_pts_detected_process.m — detected 点列预处理（每行一对 x1y1 x2y2）
 *
 * 行为：遍历每行，若列1:2与累积末点不同则加入；若列3:4非零则也加入；处理累积缓冲：
 * 如果下一行起点等于本行末点（连续段）则继续累积，否则刷出。
 */
static inline void apaBoundaryPtsDetectedProcess(const Real boundaryPts[][4],
                                                 Integer boundaryPtsN,
                                                 ApaBoundaryPreprocOutput_t *output) {
  Integer i;
  Real tempPts[APA_BOUNDARY_PREPROC_MAX_COLLINEAR][2];
  Integer tempCount;

  apaBoundaryPreprocOutputInit(output);
  if (boundaryPtsN == 0) {
    return;
  }

  tempCount = 0;
  for (i = 0; i < boundaryPtsN; i = i + 1) {
    Boolean appendStart;
    Boolean isSeg;
    Boolean continueWithNext;

    if (tempCount == 0) {
      appendStart = true;
    } else {
      Boolean equalX;
      Boolean equalY;
      equalX = apaMathEqual(tempPts[tempCount - 1][0], boundaryPts[i][0]);
      equalY = apaMathEqual(tempPts[tempCount - 1][1], boundaryPts[i][1]);
      appendStart = !(equalX && equalY);
    }
    if (appendStart) {
      if (tempCount < APA_BOUNDARY_PREPROC_MAX_COLLINEAR) {
        tempPts[tempCount][0] = boundaryPts[i][0];
        tempPts[tempCount][1] = boundaryPts[i][1];
        tempCount = tempCount + 1;
      }
    }

    isSeg = !apaBoundaryIsVirtualPoint(boundaryPts[i][2], boundaryPts[i][3]);
    if (isSeg) {
      if (tempCount < APA_BOUNDARY_PREPROC_MAX_COLLINEAR) {
        tempPts[tempCount][0] = boundaryPts[i][2];
        tempPts[tempCount][1] = boundaryPts[i][3];
        tempCount = tempCount + 1;
      }
    }

    continueWithNext = false;
    if (tempCount == 1) {

      continueWithNext = false;
    } else {
      Boolean hasNext;
      hasNext = ((i + 1) < boundaryPtsN);
      if (hasNext) {
        Boolean nextIsSeg;
        nextIsSeg = !apaBoundaryIsVirtualPoint(boundaryPts[i + 1][2], boundaryPts[i + 1][3]);
        if (nextIsSeg) {

          Boolean equalX;
          Boolean equalY;
          equalX = apaMathEqual(tempPts[tempCount - 1][0], boundaryPts[i + 1][0]);
          equalY = apaMathEqual(tempPts[tempCount - 1][1], boundaryPts[i + 1][1]);
          if (equalX && equalY) {
            continueWithNext = true;
          }
        }
      }
    }
    if (!continueWithNext) {
      apaBoundaryFlushTempBuffer((const Real(*)[2])tempPts, tempCount, output);
      tempCount = 0;
    }
  }
}

/**
 * @brief get_1st_boundary.m — 第一边界生成
 *
 * 输入：
 *  - frSpcDetectedPts[N][4]：检测点列（每行 (x1,y1,x2,y2)）
 *  - frSpcDetectedPtsN：有效行数
 *  - frSpcUndetectedCluster[M][2]：undetected cluster 点列
 *  - frSpcUndetectedClusterN：cluster 行数
 *  - boundaryComposeType：组合类型 (NONE/DETECTED/UNDETECTED/BOTH)
 *  - frSpcDetectedPtsCls[N][2]：每行 class 标记
 *  - clsDemand：过滤需求 (ALL/DYNAMIC/STATIC/OTHERS)
 * 输出：output 累积所有线段和离散点。
 *
 * MATLAB 常量：FS_CLS_PEDESTRIAN=6（动态），FS_CLS_OCC_SIGN=7（静态）。
 */
static inline void apaGet1stBoundary(const Real frSpcDetectedPts[][4],
                                     Integer frSpcDetectedPtsN,
                                     const Real frSpcUndetectedCluster[][2],
                                     Integer frSpcUndetectedClusterN,
                                     Integer boundaryComposeType,
                                     const UInteger8 frSpcDetectedPtsCls[][2],
                                     Integer clsDemand,
                                     ApaBoundaryPreprocOutput_t *output) {
  Boolean useDetected;
  Boolean useUndetected;
  const UInteger8 fsClsPedestrianVal0 = 6;
  const UInteger8 fsClsOccSignVal0 = 7;

  apaBoundaryPreprocOutputInit(output);

  if (boundaryComposeType == APA_BOUND_COMPOSE_NONE) {
    return;
  }
  useDetected = (boundaryComposeType != APA_BOUND_COMPOSE_UNDETECTED);
  useUndetected = (boundaryComposeType != APA_BOUND_COMPOSE_DETECTED);

  if (useDetected) {
    Real partPts[APA_BOUNDARY_PREPROC_MAX_PTS][4];
    Integer partN;
    Integer i;
    ApaBoundaryPreprocOutput_t detectedOut;

    partN = 0;
    if (clsDemand == APA_FUSED_BOUND_ALL) {
      Integer copyN;
      copyN = frSpcDetectedPtsN;
      if (copyN > APA_BOUNDARY_PREPROC_MAX_PTS) {
        copyN = APA_BOUNDARY_PREPROC_MAX_PTS;
      }
      for (i = 0; i < copyN; i = i + 1) {
        partPts[i][0] = frSpcDetectedPts[i][0];
        partPts[i][1] = frSpcDetectedPts[i][1];
        partPts[i][2] = frSpcDetectedPts[i][2];
        partPts[i][3] = frSpcDetectedPts[i][3];
      }
      partN = copyN;
    } else if (clsDemand == APA_FUSED_BOUND_DYNAMIC) {
      for (i = 0; i < frSpcDetectedPtsN; i = i + 1) {
        if (frSpcDetectedPtsCls[i][1] == fsClsPedestrianVal0) {
          if (partN < APA_BOUNDARY_PREPROC_MAX_PTS) {
            partPts[partN][0] = frSpcDetectedPts[i][0];
            partPts[partN][1] = frSpcDetectedPts[i][1];
            partPts[partN][2] = frSpcDetectedPts[i][2];
            partPts[partN][3] = frSpcDetectedPts[i][3];
            partN = partN + 1;
          }
        }
      }
    } else if (clsDemand == APA_FUSED_BOUND_STATIC) {
      for (i = 0; i < frSpcDetectedPtsN; i = i + 1) {
        if (frSpcDetectedPtsCls[i][1] == fsClsOccSignVal0) {
          if (partN < APA_BOUNDARY_PREPROC_MAX_PTS) {
            partPts[partN][0] = frSpcDetectedPts[i][0];
            partPts[partN][1] = frSpcDetectedPts[i][1];
            partPts[partN][2] = frSpcDetectedPts[i][2];
            partPts[partN][3] = frSpcDetectedPts[i][3];
            partN = partN + 1;
          }
        }
      }
    } else if (clsDemand == APA_FUSED_BOUND_OTHERS) {
      for (i = 0; i < frSpcDetectedPtsN; i = i + 1) {
        Boolean notPed;
        Boolean notOcc;
        notPed = (frSpcDetectedPtsCls[i][1] != fsClsPedestrianVal0);
        notOcc = (frSpcDetectedPtsCls[i][1] != fsClsOccSignVal0);
        if (notPed && notOcc) {
          if (partN < APA_BOUNDARY_PREPROC_MAX_PTS) {
            partPts[partN][0] = frSpcDetectedPts[i][0];
            partPts[partN][1] = frSpcDetectedPts[i][1];
            partPts[partN][2] = frSpcDetectedPts[i][2];
            partPts[partN][3] = frSpcDetectedPts[i][3];
            partN = partN + 1;
          }
        }
      }
    } else {
      Integer copyN;
      copyN = frSpcDetectedPtsN;
      if (copyN > APA_BOUNDARY_PREPROC_MAX_PTS) {
        copyN = APA_BOUNDARY_PREPROC_MAX_PTS;
      }
      for (i = 0; i < copyN; i = i + 1) {
        partPts[i][0] = frSpcDetectedPts[i][0];
        partPts[i][1] = frSpcDetectedPts[i][1];
        partPts[i][2] = frSpcDetectedPts[i][2];
        partPts[i][3] = frSpcDetectedPts[i][3];
      }
      partN = copyN;
    }
    apaBoundaryPtsDetectedProcess((const Real(*)[4])partPts, partN, &detectedOut);

    for (i = 0; i < detectedOut.BoundaryLineN; i = i + 1) {
      if (output->BoundaryLineN < APA_BOUNDARY_PREPROC_MAX_LINES) {
        output->BoundaryLine[output->BoundaryLineN][0] = detectedOut.BoundaryLine[i][0];
        output->BoundaryLine[output->BoundaryLineN][1] = detectedOut.BoundaryLine[i][1];
        output->BoundaryLine[output->BoundaryLineN][2] = detectedOut.BoundaryLine[i][2];
        output->BoundaryLine[output->BoundaryLineN][3] = detectedOut.BoundaryLine[i][3];
        output->BoundaryLineN = output->BoundaryLineN + 1;
      }
    }
    for (i = 0; i < detectedOut.BoundarySeparatePtsN; i = i + 1) {
      if (output->BoundarySeparatePtsN < APA_BOUNDARY_PREPROC_MAX_PTS) {
        output->BoundarySeparatePts[output->BoundarySeparatePtsN][0] =
            detectedOut.BoundarySeparatePts[i][0];
        output->BoundarySeparatePts[output->BoundarySeparatePtsN][1] =
            detectedOut.BoundarySeparatePts[i][1];
        output->BoundarySeparatePtsN = output->BoundarySeparatePtsN + 1;
      }
    }
  }

  if (useUndetected) {
    ApaBoundaryPreprocOutput_t undetectedOut;
    Integer i;
    apaBoundaryClusterUndetectedProcess(
        frSpcUndetectedCluster, frSpcUndetectedClusterN, &undetectedOut);
    for (i = 0; i < undetectedOut.BoundaryLineN; i = i + 1) {
      if (output->BoundaryLineN < APA_BOUNDARY_PREPROC_MAX_LINES) {
        output->BoundaryLine[output->BoundaryLineN][0] = undetectedOut.BoundaryLine[i][0];
        output->BoundaryLine[output->BoundaryLineN][1] = undetectedOut.BoundaryLine[i][1];
        output->BoundaryLine[output->BoundaryLineN][2] = undetectedOut.BoundaryLine[i][2];
        output->BoundaryLine[output->BoundaryLineN][3] = undetectedOut.BoundaryLine[i][3];
        output->BoundaryLineN = output->BoundaryLineN + 1;
      }
    }
    for (i = 0; i < undetectedOut.BoundarySeparatePtsN; i = i + 1) {
      if (output->BoundarySeparatePtsN < APA_BOUNDARY_PREPROC_MAX_PTS) {
        output->BoundarySeparatePts[output->BoundarySeparatePtsN][0] =
            undetectedOut.BoundarySeparatePts[i][0];
        output->BoundarySeparatePts[output->BoundarySeparatePtsN][1] =
            undetectedOut.BoundarySeparatePts[i][1];
        output->BoundarySeparatePtsN = output->BoundarySeparatePtsN + 1;
      }
    }
  }
}

#endif
