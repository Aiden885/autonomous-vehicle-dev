/**
 * @file geoInboxJudgeTypes.h
 * @brief 几何点入框判断类型定义
 * @date 2026-04-14
 */
#ifndef GEO_INBOX_JUDGE_TYPES_H
#define GEO_INBOX_JUDGE_TYPES_H

#include "baseTypes.h"

/**
 * @brief 点入框判断输入
 * @en_name GeoInboxJudgeInput
 * @cn_name 点入框判断输入
 * @tag STRUCT
 * @field name=PtX, type=Real, unit=m, desc=测试点X
 * @field name=PtY, type=Real, unit=m, desc=测试点Y
 * @field name=BoxPtsX, type=Array<Real, [4]>, unit=m, desc=矩形4顶点X
 * @field name=BoxPtsY, type=Array<Real, [4]>, unit=m, desc=矩形4顶点Y
 */
typedef struct {
  Real PtX;
  Real PtY;
  Real BoxPtsX[4];
  Real BoxPtsY[4];
} GeoInboxJudgeInput_t;

/** @brief Store four scalar box corners in an inbox-judge input. */
static inline void geoInboxJudgeSetBox(GeoInboxJudgeInput_t *input,
                                       Real pt1X,
                                       Real pt1Y,
                                       Real pt2X,
                                       Real pt2Y,
                                       Real pt3X,
                                       Real pt3Y,
                                       Real pt4X,
                                       Real pt4Y) {
  input->BoxPtsX[0] = pt1X;
  input->BoxPtsY[0] = pt1Y;
  input->BoxPtsX[1] = pt2X;
  input->BoxPtsY[1] = pt2Y;
  input->BoxPtsX[2] = pt3X;
  input->BoxPtsY[2] = pt3Y;
  input->BoxPtsX[3] = pt4X;
  input->BoxPtsY[3] = pt4Y;
}

/**
 * @brief 点入框判断输出
 * @en_name GeoInboxJudgeOutput
 * @cn_name 点入框判断输出
 * @tag STRUCT
 * @field name=Inbox, type=Boolean, unit=1, desc=是否在框内标志
 */
typedef struct {
  Boolean Inbox;
} GeoInboxJudgeOutput_t;

void geoInboxJudge(const GeoInboxJudgeInput_t *input, GeoInboxJudgeOutput_t *output);

#endif
