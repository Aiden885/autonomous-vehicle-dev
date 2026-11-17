/* MATLAB sources: DetectSlotSpace.m, geo_VT_PathPlanning.m, path_planning_alg.m */

#include "apaSlotTypes.h"
#include "apaPlanningProgress.h"
#include "apaCoordUtils.h"
#include "apaVehicleDefaults.h"
#include "apaCountUtils.h"
#include "apaHybridAStar.h"
#include "apaMathUtils.h"
#include "apaPlannerHybridParity.h"
#include "apaPlannerUtils.h"
#include "apaVehicleCollisionCheck.h"
#include "geoAxisTransTypes.h"
#include "geo2CircleLinkTypes.h"
#include "geoCircle1pt1lineTypes.h"
#include "geo2CircleLinkHTTypes.h"
#include "geoCircle2lineRTypes.h"
#include "geoCircleIntrvCheckTypes.h"
#include "geoLineIntrvCheckTypes.h"
#include "geoPosTypClassPkInTypes.h"
#include "geoRev2CircleLinkTypes.h"
#include "geoStPtSpcVtTypes.h"
#include "geoVtEntrPtDngPtTypes.h"
#include "geoVt1RFixTypes.h"
#include "geoVtPathPlanningTypes.h"
#include "geoVtPathPlanningInternal.h"
#include "apaTargetSlotConstraint.h"
#include "vehicleCollisionCheckTypes.h"
#include <math.h>
#include <stdio.h>

typedef struct {
  GeoVtPathPlanningParam_t Base;
  const GeoVtPathPlanningPolicy_t *Policy;
} GeoVtPathPlanningRunParam_t;

#ifndef GEO_VT_TARGET_PRECHECK_ENTER_LEN
#define GEO_VT_TARGET_PRECHECK_ENTER_LEN 2.5f
#endif

#ifndef GEO_VT_TARGET_PRECHECK_TRANSITION_LEN
#define GEO_VT_TARGET_PRECHECK_TRANSITION_LEN 1.0f
#endif

#ifndef GEO_VT_DEFAULT_WHEELBASE
#define GEO_VT_DEFAULT_WHEELBASE APA_VEHICLE_WHEELBASE_M
#endif

#ifndef GEO_VT_MAIN_LOOP_STAGE_MAX
#define GEO_VT_MAIN_LOOP_STAGE_MAX 30
#endif

#ifndef GEO_VT_MAIN_LOOP_INITIAL_SPACE_SUM_TH
#define GEO_VT_MAIN_LOOP_INITIAL_SPACE_SUM_TH 2.5f
#endif

#ifndef GEO_VT_MAIN_LOOP_INITIAL_SIDE_SPACE_TH
#define GEO_VT_MAIN_LOOP_INITIAL_SIDE_SPACE_TH 1.5f
#endif

#ifndef GEO_VT_MAIN_LOOP_INITIAL_FORWARD_LEN
#define GEO_VT_MAIN_LOOP_INITIAL_FORWARD_LEN 2.0f
#endif

#ifndef GEO_VT_MAIN_LOOP_RADIUS_MATCH_EPS
#define GEO_VT_MAIN_LOOP_RADIUS_MATCH_EPS 0.05f
#endif

#ifndef GEO_VT_MAIN_LOOP_TAIL_LINE_MIN
#define GEO_VT_MAIN_LOOP_TAIL_LINE_MIN 0.2f
#endif

#ifndef GEO_VT_MAIN_LOOP_CIR_LEN_MAX
#define GEO_VT_MAIN_LOOP_CIR_LEN_MAX 10.0f
#endif

#ifndef GEO_VT_MAIN_LOOP_CIR_EDX_MAX
#define GEO_VT_MAIN_LOOP_CIR_EDX_MAX 7.0f
#endif

#ifndef GEO_VT_MAIN_LOOP_CIR_EDY_MAX
#define GEO_VT_MAIN_LOOP_CIR_EDY_MAX 10.0f
#endif

#ifndef GEO_VT_MAIN_LOOP_REV_DIST
#define GEO_VT_MAIN_LOOP_REV_DIST 1.0f
#endif

#ifndef GEO_VT_MAIN_LOOP_ARC_CHECK_STEP
#define GEO_VT_MAIN_LOOP_ARC_CHECK_STEP 0.2f
#endif

#ifndef GEO_VT_MAIN_LOOP_LINE_CHECK_STEP
#define GEO_VT_MAIN_LOOP_LINE_CHECK_STEP 2.0f
#endif

#ifndef GEO_VT_MAIN_LOOP_REV_LINE_CHECK_STEP
#define GEO_VT_MAIN_LOOP_REV_LINE_CHECK_STEP 0.8f
#endif

#ifndef GEO_VT_MAIN_LOOP_ITER_MAX
#define GEO_VT_MAIN_LOOP_ITER_MAX 10
#endif

typedef struct {
  Integer GoDir[GEO_VT_MAIN_LOOP_STAGE_MAX];
  Integer TrnDir[GEO_VT_MAIN_LOOP_STAGE_MAX];
  Real Crv[GEO_VT_MAIN_LOOP_STAGE_MAX];
  Real Tht[GEO_VT_MAIN_LOOP_STAGE_MAX];
  Real Len[GEO_VT_MAIN_LOOP_STAGE_MAX];
  Real Stx[GEO_VT_MAIN_LOOP_STAGE_MAX];
  Real Sty[GEO_VT_MAIN_LOOP_STAGE_MAX];
  Real StTh[GEO_VT_MAIN_LOOP_STAGE_MAX];
  Real CcX[GEO_VT_MAIN_LOOP_STAGE_MAX];
  Real CcY[GEO_VT_MAIN_LOOP_STAGE_MAX];
  Integer StpPthTypList[GEO_VT_MAIN_LOOP_STAGE_MAX];
  Integer FailRsOut[GEO_VT_MAIN_LOOP_STAGE_MAX];
  Integer PlanCmd;
  Integer SegNum;
  Integer IniSegN;
  Integer Calcnt;
  Integer StpPthTypLN;
  Integer PlnChkCnt;
  Integer StpN;
  Integer DirS;
  Real PathLen;
  Real Pt1X;
  Real Pt1Y;
  Real PlnEdPtX;
  Real PlnEdPtY;
  Real PlnEdPtTheta;
} GeoVtPathPlanningMainLoopWork_t;

#ifndef GEO_VT_PLANNER_CASE_NONE
#define GEO_VT_PLANNER_CASE_NONE 0
#endif

#ifndef GEO_VT_PLANNER_CASE_MAIN
#define GEO_VT_PLANNER_CASE_MAIN 1
#endif

#ifndef GEO_VT_PLANNER_CASE_FRONT_FALLBACK
#define GEO_VT_PLANNER_CASE_FRONT_FALLBACK 2
#endif

#ifndef GEO_VT_PLANNER_CASE_GENERIC_FALLBACK
#define GEO_VT_PLANNER_CASE_GENERIC_FALLBACK 3
#endif

#ifndef GEO_VT_PLANNER_CASE_HYBRID_FALLBACK
#define GEO_VT_PLANNER_CASE_HYBRID_FALLBACK 4
#endif

#ifndef GEO_VT_FALLBACK_GEOM_MODE_NONE
#define GEO_VT_FALLBACK_GEOM_MODE_NONE 0
#endif

#ifndef GEO_VT_FALLBACK_GEOM_MODE_SAFE
#define GEO_VT_FALLBACK_GEOM_MODE_SAFE 1
#endif

#ifndef GEO_VT_FALLBACK_GEOM_MODE_RAW
#define GEO_VT_FALLBACK_GEOM_MODE_RAW 2
#endif

#ifndef GEO_VT_FALLBACK_WINNER_NONE
#define GEO_VT_FALLBACK_WINNER_NONE 0
#endif

#ifndef GEO_VT_FALLBACK_WINNER_GEOM
#define GEO_VT_FALLBACK_WINNER_GEOM 1
#endif

#ifndef GEO_VT_FALLBACK_WINNER_HYBRID
#define GEO_VT_FALLBACK_WINNER_HYBRID 2
#endif

#ifndef GEO_VT_FALLBACK_DECISION_NONE
#define GEO_VT_FALLBACK_DECISION_NONE 0
#endif

#ifndef GEO_VT_FALLBACK_DECISION_VALID_ONLY
#define GEO_VT_FALLBACK_DECISION_VALID_ONLY 1
#endif

#ifndef GEO_VT_FALLBACK_DECISION_STEP_COUNT
#define GEO_VT_FALLBACK_DECISION_STEP_COUNT 2
#endif

#ifndef GEO_VT_FALLBACK_DECISION_PATH_LEN
#define GEO_VT_FALLBACK_DECISION_PATH_LEN 3
#endif

#ifndef GEO_VT_FALLBACK_DECISION_SEG_NUM
#define GEO_VT_FALLBACK_DECISION_SEG_NUM 4
#endif

#ifndef GEO_VT_FALLBACK_DECISION_PLN_CHK_CNT
#define GEO_VT_FALLBACK_DECISION_PLN_CHK_CNT 5
#endif

#ifndef GEO_VT_HYBRID_COLLISION_STEP
#define GEO_VT_HYBRID_COLLISION_STEP 0.2f
#endif

#ifndef GEO_VT_HYBRID_MAX_ITER
#define GEO_VT_HYBRID_MAX_ITER 600
#endif

#ifndef GEO_VT_HYBRID_CURV_EPS
#define GEO_VT_HYBRID_CURV_EPS 1.0e-6f
#endif

#ifndef GEO_VT_HYBRID_FIT_POS_EPS
#define GEO_VT_HYBRID_FIT_POS_EPS 0.02f
#endif

#ifndef GEO_VT_HYBRID_FIT_YAW_EPS
#define GEO_VT_HYBRID_FIT_YAW_EPS 0.003f
#endif

#ifndef GEO_VT_HYBRID_GOAL_LONG_ERR
#define GEO_VT_HYBRID_GOAL_LONG_ERR 0.25f
#endif

#ifndef GEO_VT_HYBRID_GOAL_LAT_ERR
#define GEO_VT_HYBRID_GOAL_LAT_ERR 0.15f
#endif

#ifndef GEO_VT_HYBRID_GOAL_YAW_ERR
#define GEO_VT_HYBRID_GOAL_YAW_ERR 0.12f
#endif

#ifndef GEO_VT_HYBRID_MAP_MARGIN
#define GEO_VT_HYBRID_MAP_MARGIN 12.0f
#endif

#ifndef GEO_VT_CANDIDATE_PATH_EPS
#define GEO_VT_CANDIDATE_PATH_EPS 0.05f
#endif

#ifndef GEO_VT_HYBRID_OVERRIDE_STEP_DELTA_MAX
#define GEO_VT_HYBRID_OVERRIDE_STEP_DELTA_MAX 2
#endif

#ifndef GEO_VT_HYBRID_OVERRIDE_PATH_GAIN_MIN
#define GEO_VT_HYBRID_OVERRIDE_PATH_GAIN_MIN 2.0f
#endif

#ifndef GEO_VT_HYBRID_OVERRIDE_PATH_RATIO_MIN
#define GEO_VT_HYBRID_OVERRIDE_PATH_RATIO_MIN 0.15f
#endif

#ifndef GEO_VT_FRONT_BLEND_WIDE
#define GEO_VT_FRONT_BLEND_WIDE 0.15f
#endif

#ifndef GEO_VT_FRONT_BLEND_NARROW
#define GEO_VT_FRONT_BLEND_NARROW 0.08f
#endif

#ifndef GEO_VT_TYPE34_R1_SCALE
#define GEO_VT_TYPE34_R1_SCALE 1.00f
#endif

#ifndef GEO_VT_TYPE34_R2_SCALE
#define GEO_VT_TYPE34_R2_SCALE 1.00f
#endif

#ifndef GEO_VT_FRONT_DIRECT_RADIUS_EPS
#define GEO_VT_FRONT_DIRECT_RADIUS_EPS 0.05f
#endif

#ifndef GEO_VT_FRONT_REPOS_DIST
#define GEO_VT_FRONT_REPOS_DIST 1.0f
#endif

#ifndef GEO_VT_FRONT_PARTIAL_MIN
#define GEO_VT_FRONT_PARTIAL_MIN 0.5f
#endif

#ifndef GEO_VT_FRONT_TAIL_MIN
#define GEO_VT_FRONT_TAIL_MIN 0.2f
#endif

#ifndef GEO_VT_FRONT_ARC_CHECK_STEP
#define GEO_VT_FRONT_ARC_CHECK_STEP 0.2f
#endif

#ifndef GEO_VT_FRONT_DIRECT_LINE_STEP
#define GEO_VT_FRONT_DIRECT_LINE_STEP 0.2f
#endif

#ifndef GEO_VT_FRONT_REPOS_LINE_STEP
#define GEO_VT_FRONT_REPOS_LINE_STEP 0.8f
#endif

#ifndef GEO_VT_FRONT_LOOP_MAX
#define GEO_VT_FRONT_LOOP_MAX 3
#endif

#ifndef GEO_VT_FRONT_RESULT_FAIL
#define GEO_VT_FRONT_RESULT_FAIL 0
#endif

#ifndef GEO_VT_FRONT_RESULT_PARTIAL
#define GEO_VT_FRONT_RESULT_PARTIAL 1
#endif

#ifndef GEO_VT_FRONT_RESULT_DONE
#define GEO_VT_FRONT_RESULT_DONE 2
#endif

/**
 * @brief Apply an optional full-candidate policy before selection.
 * @en_name geoVtPathPlanningAcceptPolicyCandidate
 * @cn_name VT候选策略验收
 * @type module
 * @param[IN] GeoVtPathPlanningRunParam_t Param 本次策略
 * @param[IN] GeoVtPathPlanningOutput_t Candidate 几何候选
 * @param[OUT] Boolean Accepted 返回候选验收标志
 * @retval Boolean 是否允许参与选择
 * @granularity atomic
 * @tag_level1 规划
 * @source safety_only
 * @version 1.0
 * @date 2026-09-30
 * @author aide
 */
static Boolean geoVtPathPlanningAcceptPolicyCandidate(const GeoVtPathPlanningRunParam_t *param,
                                                      const GeoVtPathPlanningOutput_t *candidate) {
  Boolean accepted0;
  accepted0 = true;
  if (param->Policy != (const GeoVtPathPlanningPolicy_t *)0) {
    if (param->Policy->AcceptCandidate != (GeoVtCandidateAccept_t)0) {
      accepted0 = param->Policy->AcceptCandidate(candidate, param->Policy->UserData);
    }
  }
  return accepted0;
}

/**
 * @brief Resolve the separate slot policy of this invocation.
 * @en_name geoVtPathPlanningSlotConstraint
 * @cn_name VT本次车位规则
 * @type module
 * @param[IN] GeoVtPathPlanningRunParam_t Param 本次策略
 * @param[OUT] ApaTargetSlotConstraint_t Constraint 返回规则指针或空指针
 * @retval ApaTargetSlotConstraint_t 规则指针或空指针
 * @granularity atomic
 * @tag_level1 规划
 * @source safety_only
 * @version 1.0
 * @date 2026-09-30
 * @author aide
 */
static const ApaTargetSlotConstraint_t *
geoVtPathPlanningSlotConstraint(const GeoVtPathPlanningRunParam_t *param) {
  const ApaTargetSlotConstraint_t *constraint;
  constraint = (const ApaTargetSlotConstraint_t *)0;
  if (param->Policy != (const GeoVtPathPlanningPolicy_t *)0) {
    constraint = param->Policy->SlotConstraint;
  }
  return constraint;
}

/**
 * @brief Combine a physical interval with the separate target-slot interval.
 * @en_name geoVtPathPlanningMergeSlotInterval
 * @cn_name VT合并车位区间检查
 * @type module
 * @param[IN] ApaTargetSlotMotionCheck_t Frame 车位检查
 * @param[OUT] GeoCircleIntrvCheckOutput_t Output 合并检查
 * @granularity atomic
 * @tag_level1 规划
 * @source safety_only
 * @version 1.0
 * @date 2026-09-30
 * @author aide
 */
static void geoVtPathPlanningMergeSlotInterval(const ApaTargetSlotMotionCheck_t *frame,
                                               GeoCircleIntrvCheckOutput_t *output) {
  Boolean replace0;
  replace0 = false;
  output->CheckCount = output->CheckCount + frame->CheckCount;
  if (!frame->IsClear) {
    replace0 = (output->IntrvFlg == 0);
    if (frame->SafeLength < output->SafeLen) {
      replace0 = true;
    }
  }
  if (replace0) {
    output->IntrvFlg = 1;
    if (frame->SafeLength <= 0.0f) {
      output->IntrvFlg = 2;
    }
    output->IntrvRS = 4;
    output->SafeLen = frame->SafeLength;
    output->AvlPosX = frame->SafeX;
    output->AvlPosY = frame->SafeY;
    output->AvlPosTheta = frame->SafeTheta;
    output->CrshPosX = frame->CollisionX;
    output->CrshPosY = frame->CollisionY;
    output->CrshPosTheta = frame->CollisionTheta;
  }
}

/**
 * @brief Check a circular interval under the optional target-slot policy.
 * @en_name geoVtPathPlanningCheckCircleWithPolicy
 * @cn_name VT圆弧及车位约束检查
 * @type module
 * @param[IN] GeoCircleIntrvCheckInput_t Input 原始圆弧及地图
 * @param[IN] GeoVtPathPlanningRunParam_t Param 本次策略
 * @param[OUT] GeoCircleIntrvCheckOutput_t Output 完整区间结果
 * @granularity composite
 * @tag_level1 规划
 * @source safety_only
 * @version 1.0
 * @date 2026-09-30
 * @author aide
 */
static void geoVtPathPlanningCheckCircleWithPolicy(const GeoCircleIntrvCheckInput_t *input,
                                                   GeoCircleIntrvCheckOutput_t *output,
                                                   const GeoVtPathPlanningRunParam_t *param) {
  const ApaTargetSlotConstraint_t *constraint;
  ApaTargetSlotMotion_t motion;
  ApaTargetSlotMotionCheck_t frame;
  Real directedAngle0;
  Integer product0;
  geoCircleIntrvCheckWithScene(input, &param->Base.SceneQuery, output);
  constraint = geoVtPathPlanningSlotConstraint(param);
  if (constraint != (const ApaTargetSlotConstraint_t *)0) {
    if (constraint->Enabled) {
      product0 = input->GoDir * input->TrnDir;
      directedAngle0 = input->Pt2Theta - input->Pt1Theta;
      if (product0 < 0) {
        directedAngle0 = -directedAngle0;
      }
      if (directedAngle0 < -1.0e-4f) {
        directedAngle0 = directedAngle0 + APA_TWO_PI;
      }
      motion = (ApaTargetSlotMotion_t){0};
      motion.StartX = input->Pt1X;
      motion.StartY = input->Pt1Y;
      motion.StartTheta = input->Pt1Theta;
      motion.Direction = input->GoDir;
      if (input->Radius > GEO_EPSILON) {
        motion.Curvature = ((Real)input->TrnDir) / input->Radius;
      }
      motion.Length = fmaxf(0.0f, directedAngle0 * input->Radius);
      apaTargetSlotConstraintCheckMotion(constraint, &motion, &frame);
      geoVtPathPlanningMergeSlotInterval(&frame, output);
    }
  }
}

/**
 * @brief Check a straight interval under the optional target-slot policy.
 * @en_name geoVtPathPlanningCheckLineWithPolicy
 * @cn_name VT直线及车位约束检查
 * @type module
 * @param[IN] GeoLineIntrvCheckInput_t Input 原始直线及地图
 * @param[IN] GeoVtPathPlanningRunParam_t Param 本次策略
 * @param[OUT] GeoLineIntrvCheckOutput_t Output 完整区间结果
 * @granularity composite
 * @tag_level1 规划
 * @source safety_only
 * @version 1.0
 * @date 2026-09-30
 * @author aide
 */
static void geoVtPathPlanningCheckLineWithPolicy(const GeoLineIntrvCheckInput_t *input,
                                                 GeoLineIntrvCheckOutput_t *output,
                                                 const GeoVtPathPlanningRunParam_t *param) {
  const ApaTargetSlotConstraint_t *constraint;
  ApaTargetSlotMotion_t motion;
  ApaTargetSlotMotionCheck_t frame;
  Real dx0;
  Real dy0;
  Real along0;
  GeoCircleIntrvCheckOutput_t combined;
  geoLineIntrvCheckWithScene(input, &param->Base.SceneQuery, output);
  constraint = geoVtPathPlanningSlotConstraint(param);
  if (constraint != (const ApaTargetSlotConstraint_t *)0) {
    if (constraint->Enabled) {
      motion = (ApaTargetSlotMotion_t){0};
      dx0 = input->Pt2X - input->Pt1X;
      dy0 = input->Pt2Y - input->Pt1Y;
      along0 = (dx0 * cosf(input->Pt1Theta)) + (dy0 * sinf(input->Pt1Theta));
      motion.StartX = input->Pt1X;
      motion.StartY = input->Pt1Y;
      motion.StartTheta = input->Pt1Theta;
      motion.Direction = 1;
      if (along0 < 0.0f) {
        motion.Direction = -1;
      }
      motion.Length = sqrtf((dx0 * dx0) + (dy0 * dy0));
      apaTargetSlotConstraintCheckMotion(constraint, &motion, &frame);
      combined.IntrvFlg = output->IntrvFlg;
      combined.IntrvRS = output->IntrvRS;
      combined.SafeLen = output->SafeLen;
      combined.CheckCount = output->CheckCount;
      combined.AvlPosX = output->AvlPosX;
      combined.AvlPosY = output->AvlPosY;
      combined.AvlPosTheta = output->AvlPosTheta;
      combined.CrshPosX = output->CrshPosX;
      combined.CrshPosY = output->CrshPosY;
      combined.CrshPosTheta = output->CrshPosTheta;
      geoVtPathPlanningMergeSlotInterval(&frame, &combined);
      output->IntrvFlg = combined.IntrvFlg;
      output->IntrvRS = combined.IntrvRS;
      output->SafeLen = combined.SafeLen;
      output->CheckCount = combined.CheckCount;
      output->AvlPosX = combined.AvlPosX;
      output->AvlPosY = combined.AvlPosY;
      output->AvlPosTheta = combined.AvlPosTheta;
      output->CrshPosX = combined.CrshPosX;
      output->CrshPosY = combined.CrshPosY;
      output->CrshPosTheta = combined.CrshPosTheta;
    }
  }
}

static Boolean geoVtPathPlanningIsFrontStartType(Integer startPosTyp) {
  Boolean condType20;
  Boolean condType40;
  Boolean condType320;
  Boolean condType340;
  Boolean isFrontType0;

  condType20 = (startPosTyp == 2);
  condType40 = (startPosTyp == 4);
  condType320 = (startPosTyp == 32);
  condType340 = (startPosTyp == 34);
  isFrontType0 = condType20 || condType40 || condType320 || condType340;

  return isFrontType0;
}

static void geoVtPathPlanningBuildAnchor(const GeoVtPathPlanningInput_t *input,
                                         Real blendValue,
                                         Real *anchorX,
                                         Real *anchorY) {
  Real targetDx;
  Real targetDy;

  targetDx = input->TarX - input->EntrPtX;
  targetDy = input->TarY - input->EntrPtY;
  *anchorX = input->EntrPtX + blendValue * targetDx;
  *anchorY = input->EntrPtY + blendValue * targetDy;
}

static void geoVtPathPlanningExportCandidate(const GeoVtPathPlanningOutput_t *candidate,
                                             Integer *pathSegNum,
                                             Integer pathGoDir[APA_PATH_SEG_MAX],
                                             Integer pathTrnDir[APA_PATH_SEG_MAX],
                                             Real pathCrv[APA_PATH_SEG_MAX],
                                             Real pathTht[APA_PATH_SEG_MAX],
                                             Real pathLen[APA_PATH_SEG_MAX],
                                             Real pathStx[APA_PATH_SEG_MAX],
                                             Real pathSty[APA_PATH_SEG_MAX],
                                             Real pathStTh[APA_PATH_SEG_MAX],
                                             Real pathCC[APA_PATH_CC_MAX]);
static void geoVtPathPlanningZeroPathArrays(Integer pathGoDir[APA_PATH_SEG_MAX],
                                            Integer pathTrnDir[APA_PATH_SEG_MAX],
                                            Real pathCrv[APA_PATH_SEG_MAX],
                                            Real pathTht[APA_PATH_SEG_MAX],
                                            Real pathLen[APA_PATH_SEG_MAX],
                                            Real pathStx[APA_PATH_SEG_MAX],
                                            Real pathSty[APA_PATH_SEG_MAX],
                                            Real pathStTh[APA_PATH_SEG_MAX],
                                            Real pathCC[APA_PATH_CC_MAX]);

static Integer geoVtPathPlanningCountDiagList(const Integer stpPthTypList[APA_PATH_SEG_MAX]) {
  Integer idx;
  Integer count;
  Boolean scanning;

  idx = 0;
  count = 0;
  scanning = true;
  while ((idx < APA_PATH_SEG_MAX) && scanning) {
    if (stpPthTypList[idx] != 0) {
      count = idx + 1;
    } else {
      scanning = false;
    }
    idx = idx + 1;
  }
  return count;
}

static void geoVtPathPlanningLoadDiagLists(const Integer stpPthTypList[APA_PATH_SEG_MAX],
                                           const Integer failRsOut[APA_PATH_SEG_MAX],
                                           GeoVtPathPlanningOutput_t *output) {
  Integer idx;

  for (idx = 0; idx < APA_PATH_SEG_MAX; idx = idx + 1) {
    output->StpPthTypList[idx] = stpPthTypList[idx];
    output->FailRsOut[idx] = failRsOut[idx];
  }

  output->StpPthTyp0 = stpPthTypList[0];
  output->StpPthTyp1 = stpPthTypList[1];
  output->StpPthTypLN = geoVtPathPlanningCountDiagList(stpPthTypList);
  output->FailRs0 = failRsOut[0];
  output->FailRs1 = failRsOut[1];
}

static Real geoVtPathPlanningResolveRmin(const GeoVtPathPlanningInput_t *input,
                                         const GeoVtPathPlanningRunParam_t *param) {
  Real rmin0;
  Boolean useClassifiedSlot0;

  rmin0 = input->Rmin;
  useClassifiedSlot0 = false;
  if (param->Base.SltState == 1) {
    useClassifiedSlot0 = true;
  } else {
    if (param->Base.SltState == 3) {
      useClassifiedSlot0 = true;
    }
  }
  if (useClassifiedSlot0) {
    rmin0 = param->Base.RminClassifiedSlot;
  }
  return rmin0;
}

static Integer geoVtPathPlanningMainLoopTailIndex(const GeoVtPathPlanningMainLoopWork_t *work,
                                                  Boolean includeLastSlot) {
  Integer stageIndex0;
  Integer upperIndex0;

  if (includeLastSlot) {
    upperIndex0 = GEO_VT_MAIN_LOOP_STAGE_MAX - 1;
  } else {
    upperIndex0 = GEO_VT_MAIN_LOOP_STAGE_MAX - 2;
  }
  stageIndex0 = (GEO_VT_MAIN_LOOP_STAGE_MAX - 1) - work->SegNum;
  if (stageIndex0 > upperIndex0) {
    stageIndex0 = upperIndex0;
  }
  stageIndex0 = apaClampCount(stageIndex0, GEO_VT_MAIN_LOOP_STAGE_MAX - 1);

  return stageIndex0;
}

static void geoVtPathPlanningMainLoopWorkInit(GeoVtPathPlanningMainLoopWork_t *work,
                                              const GeoVtPathPlanningInput_t *input) {
  Integer idx;

  idx = 0;
  while (idx < GEO_VT_MAIN_LOOP_STAGE_MAX) {
    work->GoDir[idx] = 0;
    work->TrnDir[idx] = 0;
    work->Crv[idx] = 0.0f;
    work->Tht[idx] = 0.0f;
    work->Len[idx] = 0.0f;
    work->Stx[idx] = 0.0f;
    work->Sty[idx] = 0.0f;
    work->StTh[idx] = 0.0f;
    work->CcX[idx] = 0.0f;
    work->CcY[idx] = 0.0f;
    work->StpPthTypList[idx] = 0;
    work->FailRsOut[idx] = 0;
    idx = idx + 1;
  }

  work->PlanCmd = 0;
  work->SegNum = 0;
  work->IniSegN = 0;
  work->Calcnt = 0;
  work->StpPthTypLN = 0;
  work->PlnChkCnt = 0;
  work->StpN = 0;
  work->DirS = 1;
  work->PathLen = 0.0f;
  work->Pt1X = 0.0f;
  work->Pt1Y = 0.0f;
  work->PlnEdPtX = input->VehX;
  work->PlnEdPtY = input->VehY;
  work->PlnEdPtTheta = input->VehTheta;
}

static Boolean geoVtPathPlanningMainLoopResolveInitialOffsetDesired(Real targetRelY,
                                                                    Real leftSpace,
                                                                    Real rightSpace,
                                                                    Real *iniOffset) {
  Real spaceSum0;
  Real halfSpace0;
  Boolean targetOnLeft0;
  Boolean enoughSpace0;
  Boolean rightTight0;
  Boolean leftTight0;
  Boolean leftOffsetCond0;
  Boolean rightOffsetCond0;
  Boolean desired0;

  spaceSum0 = leftSpace + rightSpace;
  halfSpace0 = 0.5f * spaceSum0;
  targetOnLeft0 = (targetRelY > 0.0f);
  enoughSpace0 = (spaceSum0 > GEO_VT_MAIN_LOOP_INITIAL_SPACE_SUM_TH);
  rightTight0 = (rightSpace < GEO_VT_MAIN_LOOP_INITIAL_SIDE_SPACE_TH);
  leftTight0 = (leftSpace < GEO_VT_MAIN_LOOP_INITIAL_SIDE_SPACE_TH);
  leftOffsetCond0 = false;
  rightOffsetCond0 = false;
  if (enoughSpace0) {
    if (rightTight0) {
      leftOffsetCond0 = true;
    }
    if (leftTight0) {
      rightOffsetCond0 = true;
    }
  }
  desired0 = false;
  *iniOffset = 0.0f;

  if (targetOnLeft0) {
    if (leftOffsetCond0) {
      *iniOffset = halfSpace0 - rightSpace;
      desired0 = true;
    }
  } else {
    if (rightOffsetCond0) {
      *iniOffset = -(halfSpace0 - leftSpace);
      desired0 = true;
    }
  }

  return desired0;
}

static void geoVtPathPlanningMainLoopWriteStageSegment(GeoVtPathPlanningMainLoopWork_t *work,
                                                       Integer stageIndex,
                                                       Integer goDir,
                                                       Integer trnDir,
                                                       Real crv,
                                                       Real tht,
                                                       Real len,
                                                       Real stX,
                                                       Real stY,
                                                       Real stTheta,
                                                       Real ccX,
                                                       Real ccY) {
  Integer stageIndex0;

  stageIndex0 = apaClampCount(stageIndex, GEO_VT_MAIN_LOOP_STAGE_MAX - 1);
  work->GoDir[stageIndex0] = goDir;
  work->TrnDir[stageIndex0] = trnDir;
  work->Crv[stageIndex0] = crv;
  work->Tht[stageIndex0] = tht;
  work->Len[stageIndex0] = len;
  work->Stx[stageIndex0] = stX;
  work->Sty[stageIndex0] = stY;
  work->StTh[stageIndex0] = stTheta;
  work->CcX[stageIndex0] = ccX;
  work->CcY[stageIndex0] = ccY;
}

static void
geoVtPathPlanningMainLoopWriteTerminalReverseSegment(const GeoVtPathPlanningInput_t *input,
                                                     GeoVtPathPlanningMainLoopWork_t *work) {
  Real dx0;
  Real dy0;
  Real terminalLen0;
  Integer stageIndex0;
  Boolean canWrite0;

  dx0 = work->Pt1X - input->TarX;
  dy0 = work->Pt1Y - input->TarY;
  terminalLen0 = sqrtf((dx0 * dx0) + (dy0 * dy0));
  canWrite0 = (work->SegNum < GEO_VT_MAIN_LOOP_STAGE_MAX);
  if (canWrite0) {
    stageIndex0 = geoVtPathPlanningMainLoopTailIndex(work, true);
    geoVtPathPlanningMainLoopWriteStageSegment(work,
                                               stageIndex0,
                                               -1,
                                               0,
                                               0.0f,
                                               0.0f,
                                               terminalLen0,
                                               work->Pt1X,
                                               work->Pt1Y,
                                               input->TarTheta,
                                               0.0f,
                                               0.0f);
    work->SegNum = work->SegNum + 1;
    work->DirS = 1;
  }
}

static void geoVtPathPlanningMainLoopCopyStageToOutput(const GeoVtPathPlanningMainLoopWork_t *work,
                                                       Integer stageIndex,
                                                       Integer outputIndex,
                                                       GeoVtPathPlanningOutput_t *output) {
  Integer stageIndex0;

  stageIndex0 = apaClampCount(stageIndex, GEO_VT_MAIN_LOOP_STAGE_MAX - 1);
  output->SegGoDir[outputIndex] = work->GoDir[stageIndex0];
  output->SegTrnDir[outputIndex] = work->TrnDir[stageIndex0];
  output->SegCrv[outputIndex] = work->Crv[stageIndex0];
  output->SegTht[outputIndex] = work->Tht[stageIndex0];
  output->SegLen[outputIndex] = work->Len[stageIndex0];
  output->SegStX[outputIndex] = work->Stx[stageIndex0];
  output->SegStY[outputIndex] = work->Sty[stageIndex0];
  output->SegStTh[outputIndex] = work->StTh[stageIndex0];
  output->SegCcX[outputIndex] = work->CcX[stageIndex0];
  output->SegCcY[outputIndex] = work->CcY[stageIndex0];
}

static void geoVtPathPlanningMainLoopLoadDiagToOutput(const GeoVtPathPlanningMainLoopWork_t *work,
                                                      GeoVtPathPlanningOutput_t *output) {
  Integer idx;

  idx = 0;
  while (idx < GEO_VT_MAIN_LOOP_STAGE_MAX) {
    output->StpPthTypList[idx] = work->StpPthTypList[idx];
    output->FailRsOut[idx] = work->FailRsOut[idx];
    idx = idx + 1;
  }
  output->StpPthTypLN = work->StpPthTypLN;
  output->StpPthTyp0 = work->StpPthTypList[0];
  output->StpPthTyp1 = work->StpPthTypList[1];
  output->FailRs0 = work->FailRsOut[0];
  output->FailRs1 = work->FailRsOut[1];
  output->PlnChkCnt = work->PlnChkCnt;
}

static Integer geoVtPathPlanningMainLoopCountStpN(const GeoVtPathPlanningOutput_t *output) {
  Integer idx;
  Integer dirTmp0;
  Integer stpN0;
  Integer prod0;

  stpN0 = 0;
  if (output->SegNum > 0) {
    dirTmp0 = output->SegGoDir[0];
    if (dirTmp0 < 0) {
      stpN0 = 1;
    }
    idx = 1;
    while (idx < output->SegNum) {
      prod0 = output->SegGoDir[idx] * dirTmp0;
      if (prod0 < 0) {
        stpN0 = stpN0 + 1;
        dirTmp0 = output->SegGoDir[idx];
      }
      idx = idx + 1;
    }
  }

  return stpN0;
}

static Boolean geoVtPathPlanningMainLoopAssembleOutput(const GeoVtPathPlanningMainLoopWork_t *work,
                                                       GeoVtPathPlanningOutput_t *output) {
  Integer finalSegNum0;
  Integer idx;
  Integer tailStart0;
  Boolean planDone0;
  Boolean segCountOk0;
  Boolean outputCountOk0;
  Boolean assembleOk0;
  Boolean hasMainSeg0;
  Boolean stageCountOk0;
  Boolean planAndSegOk0;

  *output = (GeoVtPathPlanningOutput_t){0};
  finalSegNum0 = work->SegNum + work->IniSegN;
  planDone0 = (work->PlanCmd == 1);
  hasMainSeg0 = (work->SegNum > 0);
  stageCountOk0 = (finalSegNum0 <= GEO_VT_MAIN_LOOP_STAGE_MAX);
  segCountOk0 = false;
  if (hasMainSeg0) {
    if (stageCountOk0) {
      segCountOk0 = true;
    }
  }
  outputCountOk0 = (finalSegNum0 <= VT_SEG_OUT_MAX);
  planAndSegOk0 = false;
  if (planDone0) {
    if (segCountOk0) {
      planAndSegOk0 = true;
    }
  }
  assembleOk0 = false;
  if (planAndSegOk0) {
    if (outputCountOk0) {
      assembleOk0 = true;
    }
  }

  if (assembleOk0) {
    output->PlanFlg = 1;
    output->SegNum = finalSegNum0;
    idx = 0;
    while (idx < work->IniSegN) {
      geoVtPathPlanningMainLoopCopyStageToOutput(work, idx, idx, output);
      idx = idx + 1;
    }

    tailStart0 = GEO_VT_MAIN_LOOP_STAGE_MAX - work->SegNum;
    idx = 0;
    while (idx < work->SegNum) {
      Integer outputIndex0;
      Integer stageIndex0;

      outputIndex0 = work->IniSegN + idx;
      stageIndex0 = tailStart0 + idx;
      geoVtPathPlanningMainLoopCopyStageToOutput(work, stageIndex0, outputIndex0, output);
      idx = idx + 1;
    }

    output->PathLen = 0.0f;
    idx = 0;
    while (idx < output->SegNum) {
      output->PathLen = output->PathLen + output->SegLen[idx];
      idx = idx + 1;
    }
    output->StpN = geoVtPathPlanningMainLoopCountStpN(output);
    output->PlnChkCnt = work->PlnChkCnt;
    geoVtPathPlanningMainLoopLoadDiagToOutput(work, output);
  } else {
    output->PlanFlg = 0;
    output->SegNum = 0;
    output->PathLen = 0.0f;
    output->StpN = 0;
    output->PlnChkCnt = work->PlnChkCnt;
    geoVtPathPlanningMainLoopLoadDiagToOutput(work, output);
  }

  return assembleOk0;
}

static void
geoVtPathPlanningBuildTargetPrecheckCollisionInput(const GeoVtPathPlanningInput_t *input,
                                                   const GeoVtPathPlanningRunParam_t *param,
                                                   Real checkX,
                                                   Real checkY,
                                                   Real checkTheta,
                                                   VehicleCollisionCheckInput_t *collisionInput) {
  Integer idx;
  Integer boundaryLineCount0;
  Integer boundarySepCount0;
  Integer stopperCount0;
  Integer trackStopperCount0;
  Integer virtualLineCount0;

  *collisionInput = (VehicleCollisionCheckInput_t){0};
  collisionInput->CheckType = VCC_CHECK_TYPE_PLANNING;
  collisionInput->CollisionModel = VCC_COLLISION_MODEL_EIGHT_EDGES;
  collisionInput->CollisionDemand = VCC_COLLISION_DEMAND_SIMPLIFY;
  collisionInput->SlotClass = VCC_SLOT_CLASS_VERTICAL;
  collisionInput->SlotPtsValid = true;
  collisionInput->SlotPts[0] = input->SltPt1X;
  collisionInput->SlotPts[1] = input->SltPt1Y;
  collisionInput->SlotPts[2] = input->SltPt2X;
  collisionInput->SlotPts[3] = input->SltPt2Y;
  collisionInput->SlotPts[4] = input->SltPt3X;
  collisionInput->SlotPts[5] = input->SltPt3Y;
  collisionInput->SlotPts[6] = input->SltPt4X;
  collisionInput->SlotPts[7] = input->SltPt4Y;
  apaSlotOrderFromEntrance(input->SltEntr,
                           &collisionInput->SlotPts[0],
                           &collisionInput->SlotPts[1],
                           &collisionInput->SlotPts[2],
                           &collisionInput->SlotPts[3],
                           &collisionInput->SlotPts[4],
                           &collisionInput->SlotPts[5],
                           &collisionInput->SlotPts[6],
                           &collisionInput->SlotPts[7]);
  collisionInput->TarPosValid = true;
  collisionInput->TarPosX = input->TarX;
  collisionInput->TarPosY = input->TarY;
  collisionInput->TarPosTheta = input->TarTheta;
  collisionInput->VehPosX = checkX;
  collisionInput->VehPosY = checkY;
  collisionInput->VehPosTheta = checkTheta;
  collisionInput->VehLF = param->Base.VehLlF;
  collisionInput->VehLB = param->Base.VehLr;
  collisionInput->VehLL = param->Base.VehLlF + param->Base.VehLr;
  collisionInput->VehW = param->Base.VehW;
  collisionInput->VehWB = GEO_VT_DEFAULT_WHEELBASE;
  vehicleCollisionCheckLoadDefaultMargins(collisionInput);

  boundaryLineCount0 =
      apaClampCountToLimits(input->BoundaryLineNum, GEO_BOUNDARY_LINE_MAX, VCC_MAX_BOUNDARY_LINES);
  boundarySepCount0 =
      apaClampCountToLimits(input->BoundarySepPtNum, GEO_BOUNDARY_POINT_MAX, VCC_MAX_SEPARATE_PTS);
  stopperCount0 =
      apaClampCountToLimits(input->StopperPtNum, GEO_BOUNDARY_POINT_MAX, VCC_MAX_SEPARATE_PTS);
  trackStopperCount0 = apaClampCountToLimits(
      input->SlotTrackStopperNum, GEO_BOUNDARY_LINE_MAX, VCC_MAX_TRACK_STOPPER);
  virtualLineCount0 = apaClampCountToLimits(
      input->VirtualBoundaryLineNum, GEO_BOUNDARY_LINE_MAX, VCC_MAX_VIRTUAL_LINES);

  collisionInput->BoundaryLineNum = boundaryLineCount0;
  collisionInput->BoundarySepPtsNum = boundarySepCount0;
  collisionInput->SeparateStopperNum = stopperCount0;
  collisionInput->SlotTrackStopperNum = trackStopperCount0;
  collisionInput->VirtualBoundaryLineNum = virtualLineCount0;

  idx = 0;
  while (idx < boundaryLineCount0) {
    collisionInput->BoundaryLineX1[idx] = input->BoundaryLineX1[idx];
    collisionInput->BoundaryLineY1[idx] = input->BoundaryLineY1[idx];
    collisionInput->BoundaryLineX2[idx] = input->BoundaryLineX2[idx];
    collisionInput->BoundaryLineY2[idx] = input->BoundaryLineY2[idx];
    idx = idx + 1;
  }

  idx = 0;
  while (idx < boundarySepCount0) {
    collisionInput->BoundarySepPtsX[idx] = input->BoundarySepPtX[idx];
    collisionInput->BoundarySepPtsY[idx] = input->BoundarySepPtY[idx];
    idx = idx + 1;
  }

  idx = 0;
  while (idx < stopperCount0) {
    collisionInput->SeparateStopperX[idx] = input->StopperPtX[idx];
    collisionInput->SeparateStopperY[idx] = input->StopperPtY[idx];
    idx = idx + 1;
  }

  idx = 0;
  while (idx < trackStopperCount0) {
    collisionInput->SlotTrackStopperX1[idx] = input->SlotTrackStopperX1[idx];
    collisionInput->SlotTrackStopperY1[idx] = input->SlotTrackStopperY1[idx];
    collisionInput->SlotTrackStopperX2[idx] = input->SlotTrackStopperX2[idx];
    collisionInput->SlotTrackStopperY2[idx] = input->SlotTrackStopperY2[idx];
    idx = idx + 1;
  }

  idx = 0;
  while (idx < virtualLineCount0) {
    collisionInput->VirtualBoundaryLineX1[idx] = input->VirtualBoundaryP1X[idx];
    collisionInput->VirtualBoundaryLineY1[idx] = input->VirtualBoundaryP1Y[idx];
    collisionInput->VirtualBoundaryLineX2[idx] = input->VirtualBoundaryP2X[idx];
    collisionInput->VirtualBoundaryLineY2[idx] = input->VirtualBoundaryP2Y[idx];
    idx = idx + 1;
  }
}

static Boolean geoVtPathPlanningRunTargetPrecheck(const GeoVtPathPlanningInput_t *input,
                                                  const GeoVtPathPlanningRunParam_t *param,
                                                  GeoVtPathPlanningOutput_t *output) {
  const Real checkLenList[3] = {
      GEO_VT_TARGET_PRECHECK_ENTER_LEN, GEO_VT_TARGET_PRECHECK_TRANSITION_LEN, 0.0f};
  Integer checkIdx0;
  Integer failRs0;
  Boolean targetSafe0;

  checkIdx0 = 0;
  failRs0 = 0;
  targetSafe0 = true;
  while (checkIdx0 < 3) {
    if (targetSafe0) {
      Real checkX0;
      Real checkY0;
      Real checkTheta0;
      VehicleCollisionCheckInput_t collisionInput;
      VehicleCollisionCheckOutput_t collisionOutput;

      checkX0 = input->TarX + (cosf(input->TarTheta) * checkLenList[checkIdx0]);
      checkY0 = input->TarY + (sinf(input->TarTheta) * checkLenList[checkIdx0]);
      checkTheta0 = input->TarTheta;
      geoVtPathPlanningBuildTargetPrecheckCollisionInput(
          input, param, checkX0, checkY0, checkTheta0, &collisionInput);
      collisionOutput = (VehicleCollisionCheckOutput_t){0};
      apaPlanningSceneCollisionCheck(&collisionInput, &param->Base.SceneQuery, &collisionOutput);
      if (collisionOutput.IsCollision) {
        targetSafe0 = false;
        failRs0 = apaPlannerCollisionFailReason(collisionOutput.CollisionType, 0);
      }
    }
    checkIdx0 = checkIdx0 + 1;
  }

  if (!targetSafe0) {
    apaPlanningRecordFailure(
        param->Base.Diagnostics, APA_FAULT_COLLISION_AT_TARGET_SLOT, "vt_target_precheck", 5);
    output->PlanFlg = 0;
    output->SegNum = 0;
    output->PathLen = 0.0f;
    output->PlannerCase = GEO_VT_PLANNER_CASE_NONE;
    output->FailRs0 = failRs0;
    output->FailRsOut[0] = failRs0;
  }
  return targetSafe0;
}

static Boolean geoVtPathPlanningHasEntrSceneInput(const GeoVtPathPlanningInput_t *input) {
  Real edge12dx0;
  Real edge12dy0;
  Real edge23dx0;
  Real edge23dy0;
  Real edge34dx0;
  Real edge34dy0;
  Real edge41dx0;
  Real edge41dy0;
  Real edge12Len2;
  Real edge23Len2;
  Real edge34Len2;
  Real edge41Len2;
  Boolean hasEdge12;
  Boolean hasEdge23;
  Boolean hasEdge34;
  Boolean hasEdge41;
  Boolean hasSlotEdge0;

  edge12dx0 = input->SltPt2X - input->SltPt1X;
  edge12dy0 = input->SltPt2Y - input->SltPt1Y;
  edge23dx0 = input->SltPt3X - input->SltPt2X;
  edge23dy0 = input->SltPt3Y - input->SltPt2Y;
  edge34dx0 = input->SltPt4X - input->SltPt3X;
  edge34dy0 = input->SltPt4Y - input->SltPt3Y;
  edge41dx0 = input->SltPt1X - input->SltPt4X;
  edge41dy0 = input->SltPt1Y - input->SltPt4Y;
  edge12Len2 = (edge12dx0 * edge12dx0) + (edge12dy0 * edge12dy0);
  edge23Len2 = (edge23dx0 * edge23dx0) + (edge23dy0 * edge23dy0);
  edge34Len2 = (edge34dx0 * edge34dx0) + (edge34dy0 * edge34dy0);
  edge41Len2 = (edge41dx0 * edge41dx0) + (edge41dy0 * edge41dy0);
  hasEdge12 = (edge12Len2 > GEO_EPSILON);
  hasEdge23 = (edge23Len2 > GEO_EPSILON);
  hasEdge34 = (edge34Len2 > GEO_EPSILON);
  hasEdge41 = (edge41Len2 > GEO_EPSILON);
  hasSlotEdge0 = false;
  if (hasEdge12) {
    hasSlotEdge0 = true;
  }
  if (hasEdge23) {
    hasSlotEdge0 = true;
  }
  if (hasEdge34) {
    hasSlotEdge0 = true;
  }
  if (hasEdge41) {
    hasSlotEdge0 = true;
  }

  return hasSlotEdge0;
}

static void geoVtPathPlanningBuildEntrSceneInput(const GeoVtPathPlanningInput_t *input,
                                                 GeoVtEntrPtDngPtInput_t *entrIn) {
  Integer boundaryLineCount0;
  Integer boundarySepCount0;
  Integer idx;
  Integer stopperCount0;
  Integer virtualLineCount0;

  *entrIn = (GeoVtEntrPtDngPtInput_t){0};
  boundaryLineCount0 =
      apaClampCountToLimits(input->BoundaryLineNum, GEO_BOUNDARY_LINE_MAX, DGR_LINE_MAX);
  virtualLineCount0 =
      apaClampCountToLimits(input->VirtualBoundaryLineNum, GEO_BOUNDARY_LINE_MAX, DGR_LINE_MAX);
  boundarySepCount0 =
      apaClampCountToLimits(input->BoundarySepPtNum, GEO_BOUNDARY_POINT_MAX, DGR_PT_MAX);
  stopperCount0 = apaClampCountToLimits(input->StopperPtNum, GEO_BOUNDARY_POINT_MAX, DGR_PT_MAX);
  entrIn->SltPt1X = input->SltPt1X;
  entrIn->SltPt1Y = input->SltPt1Y;
  entrIn->SltPt2X = input->SltPt2X;
  entrIn->SltPt2Y = input->SltPt2Y;
  entrIn->SltPt3X = input->SltPt3X;
  entrIn->SltPt3Y = input->SltPt3Y;
  entrIn->SltPt4X = input->SltPt4X;
  entrIn->SltPt4Y = input->SltPt4Y;
  entrIn->SltEntr = input->SltEntr;
  entrIn->VehX = input->VehX;
  entrIn->VehY = input->VehY;
  entrIn->VehTheta = input->VehTheta;
  entrIn->TgtX = input->TarX;
  entrIn->TgtY = input->TarY;
  entrIn->TgtTheta = input->TarTheta;
  entrIn->Radius = input->Rmin;
  entrIn->PrkType = input->PrkType;
  entrIn->SurdType = input->SurdType;
  entrIn->AgrOfst = input->AgrOfst;
  entrIn->BoundaryLineNum = boundaryLineCount0;
  entrIn->VirtualBoundaryLineNum = virtualLineCount0;
  entrIn->BoundarySepPtNum = boundarySepCount0;
  entrIn->StopperPtNum = stopperCount0;

  for (idx = 0; idx < boundaryLineCount0; idx = idx + 1) {
    entrIn->BoundaryP1X[idx] = input->BoundaryLineX1[idx];
    entrIn->BoundaryP1Y[idx] = input->BoundaryLineY1[idx];
    entrIn->BoundaryP2X[idx] = input->BoundaryLineX2[idx];
    entrIn->BoundaryP2Y[idx] = input->BoundaryLineY2[idx];
  }

  for (idx = 0; idx < virtualLineCount0; idx = idx + 1) {
    entrIn->VirtualBoundaryP1X[idx] = input->VirtualBoundaryP1X[idx];
    entrIn->VirtualBoundaryP1Y[idx] = input->VirtualBoundaryP1Y[idx];
    entrIn->VirtualBoundaryP2X[idx] = input->VirtualBoundaryP2X[idx];
    entrIn->VirtualBoundaryP2Y[idx] = input->VirtualBoundaryP2Y[idx];
  }

  for (idx = 0; idx < boundarySepCount0; idx = idx + 1) {
    entrIn->BoundarySepPtX[idx] = input->BoundarySepPtX[idx];
    entrIn->BoundarySepPtY[idx] = input->BoundarySepPtY[idx];
  }

  for (idx = 0; idx < stopperCount0; idx = idx + 1) {
    entrIn->StopperPtX[idx] = input->StopperPtX[idx];
    entrIn->StopperPtY[idx] = input->StopperPtY[idx];
  }
}

static Real geoVtPathPlanningArcTht(Real stTheta, Real edTheta, Integer goDir) {
  Real deltaTheta;
  Real goDirValue;
  Real segTht0;
  Boolean condHasGoDir0;

  deltaTheta = apaCoordWrapToPi(edTheta - stTheta);
  goDirValue = (Real)goDir;
  condHasGoDir0 = (fabsf(goDirValue) > GEO_EPSILON);
  if (condHasGoDir0) {
    segTht0 = deltaTheta / goDirValue;
  } else {
    segTht0 = 0.0f;
  }

  return segTht0;
}

static void geoVtPathPlanningFillCircleCheck(const GeoVtPathPlanningInput_t *input,
                                             const GeoVtPathPlanningRunParam_t *param,
                                             Real cptX,
                                             Real cptY,
                                             Real radius,
                                             Real pt1X,
                                             Real pt1Y,
                                             Real pt1Theta,
                                             Real pt2X,
                                             Real pt2Y,
                                             Real pt2Theta,
                                             Integer goDir,
                                             Integer trnDir,
                                             Real checkStep,
                                             GeoCircleIntrvCheckInput_t *chkIn) {
  Integer idx;
  Integer boundaryLineCount0;
  Integer virtualLineCount0;
  Integer boundarySepCount0;
  Integer stopperCount0;
  Integer trackStopperCount0;

  *chkIn = (GeoCircleIntrvCheckInput_t){0};
  chkIn->CptX = cptX;
  chkIn->CptY = cptY;
  chkIn->Radius = radius;
  chkIn->Pt1X = pt1X;
  chkIn->Pt1Y = pt1Y;
  chkIn->Pt1Theta = pt1Theta;
  chkIn->Pt2X = pt2X;
  chkIn->Pt2Y = pt2Y;
  chkIn->Pt2Theta = pt2Theta;
  chkIn->GoDir = goDir;
  chkIn->TrnDir = trnDir;
  chkIn->CheckStep = checkStep;
  chkIn->GoalX = input->TarX;
  chkIn->GoalY = input->TarY;
  chkIn->GoalYaw = input->TarTheta;
  chkIn->SlotP1X = input->SltPt1X;
  chkIn->SlotP1Y = input->SltPt1Y;
  chkIn->SlotP2X = input->SltPt2X;
  chkIn->SlotP2Y = input->SltPt2Y;
  chkIn->SlotP3X = input->SltPt3X;
  chkIn->SlotP3Y = input->SltPt3Y;
  chkIn->SlotP4X = input->SltPt4X;
  chkIn->SlotP4Y = input->SltPt4Y;
  apaSlotOrderFromEntrance(input->SltEntr,
                           &chkIn->SlotP1X,
                           &chkIn->SlotP1Y,
                           &chkIn->SlotP2X,
                           &chkIn->SlotP2Y,
                           &chkIn->SlotP3X,
                           &chkIn->SlotP3Y,
                           &chkIn->SlotP4X,
                           &chkIn->SlotP4Y);
  chkIn->VehWidth = param->Base.VehW;
  chkIn->VehFrontOffset = param->Base.VehLlF;
  chkIn->VehRearOffset = param->Base.VehLr;
  chkIn->SlotClass = VCC_SLOT_CLASS_VERTICAL;
  chkIn->UsePlanningCheck = true;
  chkIn->DisableSlotBoundaryFallback = true;
  chkIn->ParkingOutFlag = false;
  chkIn->DirOut = 0;

  boundaryLineCount0 = apaClampCount(input->BoundaryLineNum, GEO_BOUNDARY_LINE_MAX);
  virtualLineCount0 = apaClampCountToLimits(
      input->VirtualBoundaryLineNum, GEO_BOUNDARY_LINE_MAX, VCC_MAX_VIRTUAL_LINES);
  boundarySepCount0 =
      apaClampCountToLimits(input->BoundarySepPtNum, GEO_BOUNDARY_POINT_MAX, VCC_MAX_SEPARATE_PTS);
  stopperCount0 =
      apaClampCountToLimits(input->StopperPtNum, GEO_BOUNDARY_POINT_MAX, VCC_MAX_SEPARATE_PTS);
  trackStopperCount0 = apaClampCountToLimits(
      input->SlotTrackStopperNum, GEO_BOUNDARY_LINE_MAX, VCC_MAX_TRACK_STOPPER);

  chkIn->BoundaryLineNum = boundaryLineCount0;
  idx = 0;
  while (idx < boundaryLineCount0) {
    chkIn->BoundaryLineX1[idx] = input->BoundaryLineX1[idx];
    chkIn->BoundaryLineY1[idx] = input->BoundaryLineY1[idx];
    chkIn->BoundaryLineX2[idx] = input->BoundaryLineX2[idx];
    chkIn->BoundaryLineY2[idx] = input->BoundaryLineY2[idx];
    idx = idx + 1;
  }

  chkIn->VirtualBoundaryLineNum = virtualLineCount0;
  idx = 0;
  while (idx < virtualLineCount0) {
    chkIn->VirtualBoundaryLineX1[idx] = input->VirtualBoundaryP1X[idx];
    chkIn->VirtualBoundaryLineY1[idx] = input->VirtualBoundaryP1Y[idx];
    chkIn->VirtualBoundaryLineX2[idx] = input->VirtualBoundaryP2X[idx];
    chkIn->VirtualBoundaryLineY2[idx] = input->VirtualBoundaryP2Y[idx];
    idx = idx + 1;
  }

  chkIn->BoundarySepPtNum = boundarySepCount0;
  idx = 0;
  while (idx < boundarySepCount0) {
    chkIn->BoundarySepPtX[idx] = input->BoundarySepPtX[idx];
    chkIn->BoundarySepPtY[idx] = input->BoundarySepPtY[idx];
    idx = idx + 1;
  }

  chkIn->StopperPtNum = stopperCount0;
  idx = 0;
  while (idx < stopperCount0) {
    chkIn->StopperPtX[idx] = input->StopperPtX[idx];
    chkIn->StopperPtY[idx] = input->StopperPtY[idx];
    idx = idx + 1;
  }

  chkIn->SlotTrackStopperNum = trackStopperCount0;
  idx = 0;
  while (idx < trackStopperCount0) {
    chkIn->SlotTrackStopperX1[idx] = input->SlotTrackStopperX1[idx];
    chkIn->SlotTrackStopperY1[idx] = input->SlotTrackStopperY1[idx];
    chkIn->SlotTrackStopperX2[idx] = input->SlotTrackStopperX2[idx];
    chkIn->SlotTrackStopperY2[idx] = input->SlotTrackStopperY2[idx];
    idx = idx + 1;
  }
}

static void geoVtPathPlanningFillLineCheck(const GeoVtPathPlanningInput_t *input,
                                           const GeoVtPathPlanningRunParam_t *param,
                                           Real pt1X,
                                           Real pt1Y,
                                           Real pt1Theta,
                                           Real pt2X,
                                           Real pt2Y,
                                           Real pt2Theta,
                                           Real checkStep,
                                           Integer slotClass,
                                           GeoLineIntrvCheckInput_t *chkIn) {
  Integer idx;
  Integer boundaryLineCount0;
  Integer virtualLineCount0;
  Integer boundarySepCount0;
  Integer stopperCount0;
  Integer trackStopperCount0;

  *chkIn = (GeoLineIntrvCheckInput_t){0};
  chkIn->Pt1X = pt1X;
  chkIn->Pt1Y = pt1Y;
  chkIn->Pt1Theta = pt1Theta;
  chkIn->Pt2X = pt2X;
  chkIn->Pt2Y = pt2Y;
  chkIn->Pt2Theta = pt2Theta;
  chkIn->CheckStep = checkStep;
  chkIn->GoalX = input->TarX;
  chkIn->GoalY = input->TarY;
  chkIn->GoalYaw = input->TarTheta;
  chkIn->SlotP1X = input->SltPt1X;
  chkIn->SlotP1Y = input->SltPt1Y;
  chkIn->SlotP2X = input->SltPt2X;
  chkIn->SlotP2Y = input->SltPt2Y;
  chkIn->SlotP3X = input->SltPt3X;
  chkIn->SlotP3Y = input->SltPt3Y;
  chkIn->SlotP4X = input->SltPt4X;
  chkIn->SlotP4Y = input->SltPt4Y;
  apaSlotOrderFromEntrance(input->SltEntr,
                           &chkIn->SlotP1X,
                           &chkIn->SlotP1Y,
                           &chkIn->SlotP2X,
                           &chkIn->SlotP2Y,
                           &chkIn->SlotP3X,
                           &chkIn->SlotP3Y,
                           &chkIn->SlotP4X,
                           &chkIn->SlotP4Y);
  chkIn->VehWidth = param->Base.VehW;
  chkIn->VehFrontOffset = param->Base.VehLlF;
  chkIn->VehRearOffset = param->Base.VehLr;
  chkIn->UsePlanningCheck = true;
  chkIn->DisableSlotBoundaryFallback = true;
  chkIn->ParkingOutFlag = false;
  chkIn->DirOut = 0;
  chkIn->SlotClass = slotClass;

  boundaryLineCount0 = apaClampCount(input->BoundaryLineNum, GEO_BOUNDARY_LINE_MAX);
  virtualLineCount0 = apaClampCountToLimits(
      input->VirtualBoundaryLineNum, GEO_BOUNDARY_LINE_MAX, VCC_MAX_VIRTUAL_LINES);
  boundarySepCount0 =
      apaClampCountToLimits(input->BoundarySepPtNum, GEO_BOUNDARY_POINT_MAX, VCC_MAX_SEPARATE_PTS);
  stopperCount0 =
      apaClampCountToLimits(input->StopperPtNum, GEO_BOUNDARY_POINT_MAX, VCC_MAX_SEPARATE_PTS);
  trackStopperCount0 = apaClampCountToLimits(
      input->SlotTrackStopperNum, GEO_BOUNDARY_LINE_MAX, VCC_MAX_TRACK_STOPPER);

  chkIn->BoundaryLineNum = boundaryLineCount0;
  idx = 0;
  while (idx < boundaryLineCount0) {
    chkIn->BoundaryLineX1[idx] = input->BoundaryLineX1[idx];
    chkIn->BoundaryLineY1[idx] = input->BoundaryLineY1[idx];
    chkIn->BoundaryLineX2[idx] = input->BoundaryLineX2[idx];
    chkIn->BoundaryLineY2[idx] = input->BoundaryLineY2[idx];
    idx = idx + 1;
  }

  chkIn->VirtualBoundaryLineNum = virtualLineCount0;
  idx = 0;
  while (idx < virtualLineCount0) {
    chkIn->VirtualBoundaryLineX1[idx] = input->VirtualBoundaryP1X[idx];
    chkIn->VirtualBoundaryLineY1[idx] = input->VirtualBoundaryP1Y[idx];
    chkIn->VirtualBoundaryLineX2[idx] = input->VirtualBoundaryP2X[idx];
    chkIn->VirtualBoundaryLineY2[idx] = input->VirtualBoundaryP2Y[idx];
    idx = idx + 1;
  }

  chkIn->BoundarySepPtNum = boundarySepCount0;
  idx = 0;
  while (idx < boundarySepCount0) {
    chkIn->BoundarySepPtX[idx] = input->BoundarySepPtX[idx];
    chkIn->BoundarySepPtY[idx] = input->BoundarySepPtY[idx];
    idx = idx + 1;
  }

  chkIn->SeparateStopperNum = stopperCount0;
  idx = 0;
  while (idx < stopperCount0) {
    chkIn->SeparateStopperX[idx] = input->StopperPtX[idx];
    chkIn->SeparateStopperY[idx] = input->StopperPtY[idx];
    idx = idx + 1;
  }

  chkIn->SlotTrackStopperNum = trackStopperCount0;
  idx = 0;
  while (idx < trackStopperCount0) {
    chkIn->SlotTrackStopperX1[idx] = input->SlotTrackStopperX1[idx];
    chkIn->SlotTrackStopperY1[idx] = input->SlotTrackStopperY1[idx];
    chkIn->SlotTrackStopperX2[idx] = input->SlotTrackStopperX2[idx];
    chkIn->SlotTrackStopperY2[idx] = input->SlotTrackStopperY2[idx];
    idx = idx + 1;
  }
}

static void geoVtPathPlanningMainLoopCalcRelativePose(Real targetX,
                                                      Real targetY,
                                                      Real targetTheta,
                                                      Real baseX,
                                                      Real baseY,
                                                      Real baseTheta,
                                                      Real *relX,
                                                      Real *relY,
                                                      Real *relTheta) {
  Real dx0;
  Real dy0;
  Real cosBase0;
  Real sinBase0;

  dx0 = targetX - baseX;
  dy0 = targetY - baseY;
  cosBase0 = cosf(baseTheta);
  sinBase0 = sinf(baseTheta);
  *relX = (cosBase0 * dx0) + (sinBase0 * dy0);
  *relY = (cosBase0 * dy0) - (sinBase0 * dx0);
  *relTheta = apaCoordWrapToPi(targetTheta - baseTheta);
}

static void geoVtPathPlanningMainLoopBuildSpcInput(const GeoVtPathPlanningInput_t *input,
                                                   const GeoVtPathPlanningRunParam_t *param,
                                                   Real vehX,
                                                   Real vehY,
                                                   Real vehTheta,
                                                   Real checkLen,
                                                   GeoStPtSpcVtInput_t *spcIn) {
  *spcIn = (GeoStPtSpcVtInput_t){0};
  spcIn->VehPosX = vehX;
  spcIn->VehPosY = vehY;
  spcIn->VehPosTheta = vehTheta;
  spcIn->TarPtX = input->TarX;
  spcIn->TarPtY = input->TarY;
  spcIn->TarPtTheta = input->TarTheta;
  spcIn->SltPt1X = input->SltPt1X;
  spcIn->SltPt1Y = input->SltPt1Y;
  spcIn->SltPt2X = input->SltPt2X;
  spcIn->SltPt2Y = input->SltPt2Y;
  spcIn->SltPt3X = input->SltPt3X;
  spcIn->SltPt3Y = input->SltPt3Y;
  spcIn->SltPt4X = input->SltPt4X;
  spcIn->SltPt4Y = input->SltPt4Y;
  spcIn->SltEntr = input->SltEntr;
  spcIn->CheckLen = checkLen;
  spcIn->SltType = 2;
  spcIn->FallbackSpace = param->Base.FallbackSpace;
  spcIn->UseCollisionCheck = true;
  geoVtPathPlanningBuildTargetPrecheckCollisionInput(
      input, param, vehX, vehY, vehTheta, &spcIn->VccTemplate);
}

static Boolean geoVtPathPlanningMainLoopRunInitialOffset(const GeoVtPathPlanningInput_t *input,
                                                         const GeoVtPathPlanningRunParam_t *param,
                                                         GeoVtPathPlanningMainLoopWork_t *work,
                                                         GeoStPtSpcVtOutput_t *spcOut) {
  GeoStPtSpcVtInput_t spcIn;
  Geo2CircleLinkHTInput_t linkIn;
  Geo2CircleLinkHTOutput_t linkOut;
  GeoCircleIntrvCheckInput_t arcChkIn;
  GeoCircleIntrvCheckOutput_t arcChkOut;
  Real relX0;
  Real relY0;
  Real relTheta0;
  Real checkLen0;
  Real iniOffset0;
  Real rmin0;
  Real len0;
  Integer turnDir0;
  Boolean desired0;
  Boolean offsetOk0;
  Boolean linkOk0;
  Boolean firstArcOk0;
  Boolean secondArcOk0;
  Boolean rminValid0;
  Boolean desiredAndRmin0;

  relX0 = 0.0f;
  relY0 = 0.0f;
  relTheta0 = 0.0f;
  geoVtPathPlanningMainLoopCalcRelativePose(input->TarX,
                                            input->TarY,
                                            input->TarTheta,
                                            work->PlnEdPtX,
                                            work->PlnEdPtY,
                                            work->PlnEdPtTheta,
                                            &relX0,
                                            &relY0,
                                            &relTheta0);
  checkLen0 = 3.0f + relX0;
  geoVtPathPlanningMainLoopBuildSpcInput(
      input, param, work->PlnEdPtX, work->PlnEdPtY, work->PlnEdPtTheta, checkLen0, &spcIn);
  *spcOut = (GeoStPtSpcVtOutput_t){0};
  geoStPtSpcVtWithScene(&spcIn, &param->Base.SceneQuery, spcOut);
  work->PlnChkCnt = work->PlnChkCnt + spcOut->CheckCount;

  desired0 = geoVtPathPlanningMainLoopResolveInitialOffsetDesired(
      relY0, spcOut->LeftSpace, spcOut->RightSpace, &iniOffset0);
  rmin0 = input->Rmin;
  offsetOk0 = false;
  linkOk0 = false;
  firstArcOk0 = false;
  rminValid0 = (rmin0 > GEO_EPSILON);
  desiredAndRmin0 = false;
  if (desired0) {
    if (rminValid0) {
      desiredAndRmin0 = true;
    }
  }

  if (desiredAndRmin0) {
    turnDir0 = apaPlannerHybridSign(iniOffset0);
    if (turnDir0 != 0) {
      linkIn = (Geo2CircleLinkHTInput_t){0};
      linkOut = (Geo2CircleLinkHTOutput_t){0};
      linkIn.Pt1X = work->PlnEdPtX;
      linkIn.Pt1Y = work->PlnEdPtY;
      linkIn.Pt1Theta = work->PlnEdPtTheta;
      linkIn.Pt2X = work->PlnEdPtX - (iniOffset0 * sinf(work->PlnEdPtTheta)) +
                    (GEO_VT_MAIN_LOOP_INITIAL_FORWARD_LEN * cosf(work->PlnEdPtTheta));
      linkIn.Pt2Y = work->PlnEdPtY + (iniOffset0 * cosf(work->PlnEdPtTheta)) +
                    (GEO_VT_MAIN_LOOP_INITIAL_FORWARD_LEN * sinf(work->PlnEdPtTheta));
      linkIn.Pt2Theta = work->PlnEdPtTheta;
      linkIn.Dir = turnDir0;
      linkIn.R1 = rmin0;
      linkIn.R2 = rmin0;
      geo2CircleLinkHT(&linkIn, &linkOut);
      linkOk0 = (linkOut.FailFlag == 0);
    }
  }

  if (linkOk0) {
    geoVtPathPlanningFillCircleCheck(input,
                                     param,
                                     linkOut.Ct1X,
                                     linkOut.Ct1Y,
                                     rmin0,
                                     linkIn.Pt1X,
                                     linkIn.Pt1Y,
                                     linkIn.Pt1Theta,
                                     linkOut.St2X,
                                     linkOut.St2Y,
                                     linkOut.St2Theta,
                                     1,
                                     linkOut.TrnD1,
                                     0.2f,
                                     &arcChkIn);
    arcChkOut = (GeoCircleIntrvCheckOutput_t){0};
    geoVtPathPlanningCheckCircleWithPolicy(&arcChkIn, &arcChkOut, param);
    work->PlnChkCnt = work->PlnChkCnt + arcChkOut.CheckCount;
    firstArcOk0 = (arcChkOut.IntrvFlg == 0);
    if (firstArcOk0) {
      len0 = apaMathResolvePositiveOrFallback(arcChkOut.SafeLen, linkOut.Len1);
      geoVtPathPlanningMainLoopWriteStageSegment(work,
                                                 0,
                                                 1,
                                                 linkOut.TrnD1,
                                                 ((Real)linkOut.TrnD1) / rmin0,
                                                 linkOut.St2Theta - linkIn.Pt1Theta,
                                                 len0,
                                                 linkIn.Pt1X,
                                                 linkIn.Pt1Y,
                                                 linkIn.Pt1Theta,
                                                 linkOut.Ct1X,
                                                 linkOut.Ct1Y);
    }
  }

  if (firstArcOk0) {
    geoVtPathPlanningFillCircleCheck(input,
                                     param,
                                     linkOut.Ct2X,
                                     linkOut.Ct2Y,
                                     rmin0,
                                     linkOut.St2X,
                                     linkOut.St2Y,
                                     linkOut.St2Theta,
                                     linkOut.St3X,
                                     linkOut.St3Y,
                                     linkOut.St3Theta,
                                     1,
                                     linkOut.TrnD2,
                                     0.2f,
                                     &arcChkIn);
    arcChkOut = (GeoCircleIntrvCheckOutput_t){0};
    geoVtPathPlanningCheckCircleWithPolicy(&arcChkIn, &arcChkOut, param);
    work->PlnChkCnt = work->PlnChkCnt + arcChkOut.CheckCount;
    secondArcOk0 = (arcChkOut.IntrvFlg == 0);
    if (secondArcOk0) {
      len0 = apaMathResolvePositiveOrFallback(arcChkOut.SafeLen, linkOut.Len2);
      geoVtPathPlanningMainLoopWriteStageSegment(work,
                                                 1,
                                                 1,
                                                 linkOut.TrnD2,
                                                 ((Real)linkOut.TrnD2) / rmin0,
                                                 linkOut.St3Theta - linkOut.St2Theta,
                                                 len0,
                                                 linkOut.St2X,
                                                 linkOut.St2Y,
                                                 linkOut.St2Theta,
                                                 linkOut.Ct2X,
                                                 linkOut.Ct2Y);
      work->IniSegN = 2;
      work->PlnEdPtX = linkOut.St3X;
      work->PlnEdPtY = linkOut.St3Y;
      work->PlnEdPtTheta = linkOut.St3Theta;
      offsetOk0 = true;
    }
  }

  (void)relTheta0;
  return offsetOk0;
}

static Boolean geoVtPathPlanningMainLoopIsEvenPathType(Integer pathType) {
  Boolean isEven0;

  switch (pathType) {
  case 2:
  case 4:
  case 6:
  case 8:
  case 9:
  case 13:
  case 15:
  case 32:
  case 34:
  case 36:
  case 38:
  case 39:
  case 43:
  case 45:
    isEven0 = true;
    break;
  default:
    isEven0 = false;
    break;
  }

  return isEven0;
}

static Boolean geoVtPathPlanningMainLoopIsOddPathType(Integer pathType) {
  Boolean isOdd0;

  switch (pathType) {
  case 1:
  case 3:
  case 5:
  case 7:
  case 10:
  case 11:
  case 12:
  case 14:
  case 16:
  case 31:
  case 33:
  case 35:
  case 37:
  case 40:
  case 41:
  case 42:
  case 44:
  case 46:
    isOdd0 = true;
    break;
  default:
    isOdd0 = false;
    break;
  }

  return isOdd0;
}

static void geoVtPathPlanningMainLoopMarkFailure(GeoVtPathPlanningMainLoopWork_t *work,
                                                 Integer failReason) {
  work->PlanCmd = 5;
  if (failReason > 0) {
    if (work->FailRsOut[0] == 0) {
      work->FailRsOut[0] = failReason;
    }
  }
}

static Integer geoVtPathPlanningMainLoopTailHeadIndex(const GeoVtPathPlanningMainLoopWork_t *work) {
  Integer stageIndex0;

  stageIndex0 = GEO_VT_MAIN_LOOP_STAGE_MAX - work->SegNum;
  stageIndex0 = apaClampCount(stageIndex0, GEO_VT_MAIN_LOOP_STAGE_MAX - 1);

  return stageIndex0;
}

static Boolean
geoVtPathPlanningMainLoopWriteReverseArcSegment(GeoVtPathPlanningMainLoopWork_t *work,
                                                Integer goDir,
                                                Integer trnDir,
                                                Real radius,
                                                Real len,
                                                Real stX,
                                                Real stY,
                                                Real stTheta,
                                                Real ccX,
                                                Real ccY) {
  Integer stageIndex0;
  Integer nextIndex0;
  Real crv0;
  Real tht0;
  Boolean canWrite0;

  canWrite0 = (work->SegNum < GEO_VT_MAIN_LOOP_STAGE_MAX);
  if (canWrite0) {
    stageIndex0 = geoVtPathPlanningMainLoopTailIndex(work, false);
    nextIndex0 = stageIndex0 + 1;
    crv0 = 0.0f;
    if (fabsf(radius) > GEO_EPSILON) {
      crv0 = ((Real)trnDir) / radius;
    }
    tht0 = work->StTh[nextIndex0] - stTheta;
    geoVtPathPlanningMainLoopWriteStageSegment(
        work, stageIndex0, goDir, trnDir, crv0, tht0, len, stX, stY, stTheta, ccX, ccY);
    work->SegNum = work->SegNum + 1;
  }

  return canWrite0;
}

static Boolean
geoVtPathPlanningMainLoopWriteReverseLineSegment(GeoVtPathPlanningMainLoopWork_t *work,
                                                 Integer goDir,
                                                 Real len,
                                                 Real stX,
                                                 Real stY,
                                                 Real stTheta) {
  Integer stageIndex0;
  Boolean canWrite0;

  canWrite0 = (work->SegNum < GEO_VT_MAIN_LOOP_STAGE_MAX);
  if (canWrite0) {
    stageIndex0 = geoVtPathPlanningMainLoopTailIndex(work, false);
    geoVtPathPlanningMainLoopWriteStageSegment(
        work, stageIndex0, goDir, 0, 0.0f, 0.0f, len, stX, stY, stTheta, 0.0f, 0.0f);
    work->SegNum = work->SegNum + 1;
  }

  return canWrite0;
}

static Boolean geoVtPathPlanningMainLoopWriteLineToTail(
    GeoVtPathPlanningMainLoopWork_t *work, Integer goDir, Real stX, Real stY, Real stTheta) {
  Integer stageIndex0;
  Integer nextIndex0;
  Real dx0;
  Real dy0;
  Real len0;
  Boolean writeOk0;

  stageIndex0 = geoVtPathPlanningMainLoopTailIndex(work, false);
  nextIndex0 = stageIndex0 + 1;
  dx0 = stX - work->Stx[nextIndex0];
  dy0 = stY - work->Sty[nextIndex0];
  len0 = sqrtf((dx0 * dx0) + (dy0 * dy0));
  writeOk0 = geoVtPathPlanningMainLoopWriteReverseLineSegment(work, goDir, len0, stX, stY, stTheta);

  return writeOk0;
}

static Boolean
geoVtPathPlanningMainLoopAppendTailLineIfNeeded(const GeoVtPathPlanningInput_t *input,
                                                const GeoVtPathPlanningRunParam_t *param,
                                                GeoVtPathPlanningMainLoopWork_t *work,
                                                Real targetX,
                                                Real targetY,
                                                Real targetTheta,
                                                Real relX) {
  Integer goDir0;
  Integer stageIndex0;
  Integer nextIndex0;
  Boolean needLine0;
  Boolean appendOk0;
  GeoLineIntrvCheckInput_t lineChkIn;
  GeoLineIntrvCheckOutput_t lineChkOut;

  needLine0 = (fabsf(relX) > GEO_VT_MAIN_LOOP_TAIL_LINE_MIN);
  appendOk0 = true;
  if (needLine0) {
    goDir0 = apaPlannerHybridSign(relX);
    if (goDir0 == 0) {
      goDir0 = 1;
    }
    stageIndex0 = geoVtPathPlanningMainLoopTailIndex(work, false);
    nextIndex0 = apaClampCount(stageIndex0 + 1, GEO_VT_MAIN_LOOP_STAGE_MAX - 1);
    geoVtPathPlanningFillLineCheck(input,
                                   param,
                                   work->Stx[nextIndex0],
                                   work->Sty[nextIndex0],
                                   work->StTh[nextIndex0],
                                   targetX,
                                   targetY,
                                   targetTheta,
                                   GEO_VT_MAIN_LOOP_LINE_CHECK_STEP,
                                   VCC_SLOT_CLASS_VERTICAL,
                                   &lineChkIn);
    lineChkOut = (GeoLineIntrvCheckOutput_t){0};
    geoVtPathPlanningCheckLineWithPolicy(&lineChkIn, &lineChkOut, param);
    work->PlnChkCnt = work->PlnChkCnt + lineChkOut.CheckCount;
    if (lineChkOut.IntrvFlg == 0) {
      appendOk0 =
          geoVtPathPlanningMainLoopWriteLineToTail(work, goDir0, targetX, targetY, targetTheta);
      if (!appendOk0) {
        geoVtPathPlanningMainLoopMarkFailure(work, 7);
      }
    } else {
      appendOk0 = false;
      if (lineChkOut.IntrvFlg == 2) {
        geoVtPathPlanningMainLoopMarkFailure(work, 8);
      } else {
        geoVtPathPlanningMainLoopMarkFailure(work, 7);
      }
    }
  }

  return appendOk0;
}

static void geoVtPathPlanningMainLoopCheckCircle(const GeoVtPathPlanningInput_t *input,
                                                 const GeoVtPathPlanningRunParam_t *param,
                                                 GeoVtPathPlanningMainLoopWork_t *work,
                                                 Real cptX,
                                                 Real cptY,
                                                 Real radius,
                                                 Real stX,
                                                 Real stY,
                                                 Real stTheta,
                                                 Real edX,
                                                 Real edY,
                                                 Real edTheta,
                                                 Integer goDir,
                                                 Integer trnDir,
                                                 GeoCircleIntrvCheckOutput_t *chkOut) {
  GeoCircleIntrvCheckInput_t chkIn;

  geoVtPathPlanningFillCircleCheck(input,
                                   param,
                                   cptX,
                                   cptY,
                                   radius,
                                   stX,
                                   stY,
                                   stTheta,
                                   edX,
                                   edY,
                                   edTheta,
                                   goDir,
                                   trnDir,
                                   GEO_VT_MAIN_LOOP_ARC_CHECK_STEP,
                                   &chkIn);
  *chkOut = (GeoCircleIntrvCheckOutput_t){0};
  geoVtPathPlanningCheckCircleWithPolicy(&chkIn, chkOut, param);
  work->PlnChkCnt = work->PlnChkCnt + chkOut->CheckCount;
}

static void geoVtPathPlanningMainLoopCheckLine(const GeoVtPathPlanningInput_t *input,
                                               const GeoVtPathPlanningRunParam_t *param,
                                               GeoVtPathPlanningMainLoopWork_t *work,
                                               Real stX,
                                               Real stY,
                                               Real stTheta,
                                               Real edX,
                                               Real edY,
                                               Real edTheta,
                                               Real checkStep,
                                               Integer slotClass,
                                               GeoLineIntrvCheckOutput_t *chkOut) {
  GeoLineIntrvCheckInput_t chkIn;

  geoVtPathPlanningFillLineCheck(
      input, param, stX, stY, stTheta, edX, edY, edTheta, checkStep, slotClass, &chkIn);
  *chkOut = (GeoLineIntrvCheckOutput_t){0};
  geoVtPathPlanningCheckLineWithPolicy(&chkIn, chkOut, param);
  work->PlnChkCnt = work->PlnChkCnt + chkOut->CheckCount;
}

static Integer
geoVtPathPlanningMainLoopAppendArcFromCheck(GeoVtPathPlanningMainLoopWork_t *work,
                                            const GeoCircleIntrvCheckOutput_t *chkOut,
                                            Integer goDir,
                                            Integer trnDir,
                                            Real radius,
                                            Real fallbackLen,
                                            Real edX,
                                            Real edY,
                                            Real edTheta,
                                            Real ccX,
                                            Real ccY) {
  Real len0;
  Boolean writeOk0;
  Integer appendResult0;

  appendResult0 = 0;
  if ((chkOut->IntrvFlg == 1) && !apaPlanningMotionHasProgress(chkOut->SafeLen)) {
    geoVtPathPlanningMainLoopMarkFailure(work, 9);
    return 3;
  }
  if (chkOut->IntrvFlg == 2) {
    geoVtPathPlanningMainLoopMarkFailure(work, 8);
    appendResult0 = 3;
  } else {
    if (chkOut->IntrvFlg == 0) {
      len0 = apaMathResolvePositiveOrFallback(chkOut->SafeLen, fallbackLen);
      writeOk0 = geoVtPathPlanningMainLoopWriteReverseArcSegment(
          work, goDir, trnDir, radius, len0, edX, edY, edTheta, ccX, ccY);
      if (writeOk0) {
        appendResult0 = 1;
      } else {
        geoVtPathPlanningMainLoopMarkFailure(work, 7);
      }
    } else {
      writeOk0 = geoVtPathPlanningMainLoopWriteReverseArcSegment(work,
                                                                 goDir,
                                                                 trnDir,
                                                                 radius,
                                                                 chkOut->SafeLen,
                                                                 chkOut->AvlPosX,
                                                                 chkOut->AvlPosY,
                                                                 chkOut->AvlPosTheta,
                                                                 ccX,
                                                                 ccY);
      if (writeOk0) {
        work->PlanCmd = 2;
        appendResult0 = 2;
      } else {
        geoVtPathPlanningMainLoopMarkFailure(work, 7);
      }
    }
  }

  return appendResult0;
}

static Boolean geoVtPathPlanningMainLoopTryOneRFix(const GeoVtPathPlanningInput_t *input,
                                                   const GeoVtPathPlanningRunParam_t *param,
                                                   GeoVtPathPlanningMainLoopWork_t *work,
                                                   Real pendX,
                                                   Real pendY,
                                                   Real pendTheta,
                                                   Real rd1,
                                                   Real rd2) {
  GeoVt1RFixInput_t fixIn;
  GeoVt1RFixOutput_t fixOut;
  Integer anchorCount0;
  Integer anchorIdx0;
  Integer stageIndex0;
  Integer fixedStart0;
  Boolean fixOk0;
  Boolean canTry0;

  canTry0 = (work->SegNum >= 2);
  fixOk0 = false;
  if (canTry0) {
    fixIn = (GeoVt1RFixInput_t){0};
    fixOut = (GeoVt1RFixOutput_t){0};
    fixIn.P0X = work->Stx[GEO_VT_MAIN_LOOP_STAGE_MAX - 2];
    fixIn.P0Y = work->Sty[GEO_VT_MAIN_LOOP_STAGE_MAX - 2];
    fixIn.P0Theta = work->StTh[GEO_VT_MAIN_LOOP_STAGE_MAX - 2];
    fixIn.P1X = work->Stx[GEO_VT_MAIN_LOOP_STAGE_MAX - 1];
    fixIn.P1Y = work->Sty[GEO_VT_MAIN_LOOP_STAGE_MAX - 1];
    fixIn.P1Theta = work->StTh[GEO_VT_MAIN_LOOP_STAGE_MAX - 1];
    fixIn.CenterX = work->CcX[GEO_VT_MAIN_LOOP_STAGE_MAX - 2];
    fixIn.CenterY = work->CcY[GEO_VT_MAIN_LOOP_STAGE_MAX - 2];
    fixIn.TurnDir = work->TrnDir[GEO_VT_MAIN_LOOP_STAGE_MAX - 2];
    fixIn.Curve = work->Crv[GEO_VT_MAIN_LOOP_STAGE_MAX - 2];
    fixIn.PendX = pendX;
    fixIn.PendY = pendY;
    fixIn.PendTheta = pendTheta;
    fixIn.Rd1 = rd1;
    fixIn.Rd2 = rd2;
    fixIn.UseFullSearch = true;
    geoVtPathPlanningBuildTargetPrecheckCollisionInput(
        input, param, fixIn.P1X, fixIn.P1Y, fixIn.P1Theta, &fixIn.VccTemplate);
    geoVtPathPlanningFillCircleCheck(input,
                                     param,
                                     fixIn.CenterX,
                                     fixIn.CenterY,
                                     rd1,
                                     fixIn.P0X,
                                     fixIn.P0Y,
                                     fixIn.P0Theta,
                                     fixIn.P1X,
                                     fixIn.P1Y,
                                     fixIn.P1Theta,
                                     1,
                                     fixIn.TurnDir,
                                     GEO_VT_MAIN_LOOP_ARC_CHECK_STEP,
                                     &fixIn.IntrvTemplate);
    geoVt1RFix(&fixIn, &fixOut);
    fixOk0 = (fixOut.Success == 1);
  }

  if (fixOk0) {
    anchorCount0 = 4;
    fixedStart0 = GEO_VT_MAIN_LOOP_STAGE_MAX - anchorCount0 - 1;
    work->SegNum = 1;
    anchorIdx0 = 0;
    while (anchorIdx0 < anchorCount0) {
      stageIndex0 = fixedStart0 + anchorIdx0;
      geoVtPathPlanningMainLoopWriteStageSegment(work,
                                                 stageIndex0,
                                                 fixOut.SegGoDir[anchorIdx0],
                                                 fixOut.SegTurnDir[anchorIdx0],
                                                 fixOut.SegCurve[anchorIdx0],
                                                 fixOut.SegDTheta[anchorIdx0],
                                                 fixOut.SegLength[anchorIdx0],
                                                 fixOut.AnchorX[anchorIdx0],
                                                 fixOut.AnchorY[anchorIdx0],
                                                 fixOut.AnchorTheta[anchorIdx0],
                                                 fixOut.SegCenterX[anchorIdx0],
                                                 fixOut.SegCenterY[anchorIdx0]);
      work->SegNum = work->SegNum + 1;
      anchorIdx0 = anchorIdx0 + 1;
    }
    work->PlanCmd = 1;
  }

  return fixOk0;
}

static void geoVtPathPlanningMainLoopRunCircleLine(const GeoVtPathPlanningInput_t *input,
                                                   const GeoVtPathPlanningRunParam_t *param,
                                                   GeoVtPathPlanningMainLoopWork_t *work,
                                                   const GeoCircle1pt1lineOutput_t *circleOut,
                                                   Integer circleCheckGoDir,
                                                   Integer circleOutGoDir,
                                                   Real targetX,
                                                   Real targetY,
                                                   Real targetTheta) {
  GeoCircleIntrvCheckOutput_t arcChkOut;
  Integer appendResult0;
  Boolean tailOk0;

  geoVtPathPlanningMainLoopCheckCircle(input,
                                       param,
                                       work,
                                       circleOut->CptX,
                                       circleOut->CptY,
                                       fabsf(circleOut->Radius),
                                       work->Stx[geoVtPathPlanningMainLoopTailHeadIndex(work)],
                                       work->Sty[geoVtPathPlanningMainLoopTailHeadIndex(work)],
                                       work->StTh[geoVtPathPlanningMainLoopTailHeadIndex(work)],
                                       circleOut->LnPtX,
                                       circleOut->LnPtY,
                                       circleOut->LnPtTheta,
                                       circleCheckGoDir,
                                       circleOut->TrnDir,
                                       &arcChkOut);
  appendResult0 = geoVtPathPlanningMainLoopAppendArcFromCheck(work,
                                                              &arcChkOut,
                                                              circleOutGoDir,
                                                              circleOut->TrnDir,
                                                              fabsf(circleOut->Radius),
                                                              circleOut->ArcLen,
                                                              circleOut->LnPtX,
                                                              circleOut->LnPtY,
                                                              circleOut->LnPtTheta,
                                                              circleOut->CptX,
                                                              circleOut->CptY);
  if (appendResult0 == 1) {
    tailOk0 = geoVtPathPlanningMainLoopAppendTailLineIfNeeded(
        input, param, work, targetX, targetY, targetTheta, circleOut->LnPtRX);
    if (tailOk0) {
      work->PlanCmd = 1;
    } else {
      geoVtPathPlanningMainLoopMarkFailure(work, 7);
    }
  }
}

static void geoVtPathPlanningMainLoopRunTwoCircleLink(const GeoVtPathPlanningInput_t *input,
                                                      const GeoVtPathPlanningRunParam_t *param,
                                                      GeoVtPathPlanningMainLoopWork_t *work,
                                                      Real stX,
                                                      Real stY,
                                                      Real stTheta,
                                                      Real edX,
                                                      Real edY,
                                                      Real edTheta,
                                                      Integer pathType,
                                                      Integer linkDir,
                                                      Real radiusScale,
                                                      Integer failReasonWhenUnavailable) {
  Geo2CircleLinkInput_t linkIn;
  Geo2CircleLinkOutput_t linkOut;
  GeoCircleIntrvCheckOutput_t arcChkOut;
  Real radius0;
  Integer appendResult0;
  Boolean tailOk0;

  radius0 = input->Rmin * radiusScale;
  linkIn = (Geo2CircleLinkInput_t){0};
  linkOut = (Geo2CircleLinkOutput_t){0};
  linkIn.Pt1X = stX;
  linkIn.Pt1Y = stY;
  linkIn.Pt1Theta = stTheta;
  linkIn.Pt2X = edX;
  linkIn.Pt2Y = edY;
  linkIn.Pt2Theta = edTheta;
  linkIn.Dir = linkDir;
  linkIn.R1 = radius0;
  linkIn.R2 = radius0;
  (void)pathType;
  geo2CircleLink(&linkIn, &linkOut);

  if (linkOut.PlnFlg == 1) {
    geoVtPathPlanningMainLoopCheckCircle(input,
                                         param,
                                         work,
                                         linkOut.Ct1X,
                                         linkOut.Ct1Y,
                                         radius0,
                                         stX,
                                         stY,
                                         stTheta,
                                         linkOut.St2X,
                                         linkOut.St2Y,
                                         linkOut.St2Theta,
                                         linkDir,
                                         linkOut.TrnD1,
                                         &arcChkOut);
    appendResult0 = geoVtPathPlanningMainLoopAppendArcFromCheck(work,
                                                                &arcChkOut,
                                                                -linkDir,
                                                                linkOut.TrnD1,
                                                                radius0,
                                                                linkOut.Len1,
                                                                linkOut.St2X,
                                                                linkOut.St2Y,
                                                                linkOut.St2Theta,
                                                                linkOut.Ct1X,
                                                                linkOut.Ct1Y);
    if (appendResult0 == 1) {
      geoVtPathPlanningMainLoopCheckCircle(input,
                                           param,
                                           work,
                                           linkOut.Ct2X,
                                           linkOut.Ct2Y,
                                           radius0,
                                           linkOut.St2X,
                                           linkOut.St2Y,
                                           linkOut.St2Theta,
                                           linkOut.St3X,
                                           linkOut.St3Y,
                                           linkOut.St3Theta,
                                           linkDir,
                                           linkOut.TrnD2,
                                           &arcChkOut);
      appendResult0 = geoVtPathPlanningMainLoopAppendArcFromCheck(work,
                                                                  &arcChkOut,
                                                                  -linkDir,
                                                                  linkOut.TrnD2,
                                                                  radius0,
                                                                  linkOut.Len2,
                                                                  linkOut.St3X,
                                                                  linkOut.St3Y,
                                                                  linkOut.St3Theta,
                                                                  linkOut.Ct2X,
                                                                  linkOut.Ct2Y);
      if (appendResult0 == 1) {
        tailOk0 = geoVtPathPlanningMainLoopAppendTailLineIfNeeded(
            input, param, work, edX, edY, edTheta, linkOut.St3rX);
        if (tailOk0) {
          work->PlanCmd = 1;
        } else {
          geoVtPathPlanningMainLoopMarkFailure(work, 7);
        }
      }
    }
  } else {
    geoVtPathPlanningMainLoopMarkFailure(work, failReasonWhenUnavailable);
  }
}

static void geoVtPathPlanningMainLoopRunReverseTwoCircle(const GeoVtPathPlanningInput_t *input,
                                                         const GeoVtPathPlanningRunParam_t *param,
                                                         GeoVtPathPlanningMainLoopWork_t *work,
                                                         Real stX,
                                                         Real stY,
                                                         Real stTheta,
                                                         Real edX,
                                                         Real edY,
                                                         Real edTheta,
                                                         Integer pathType,
                                                         Integer linkDir,
                                                         Real radius,
                                                         Integer geoFailReason) {
  GeoRev2CircleLinkInput_t linkIn;
  GeoRev2CircleLinkOutput_t linkOut;
  GeoCircleIntrvCheckOutput_t arcChkOut;
  Integer appendResult0;
  Boolean tailOk0;
  Boolean fixTried0;
  Boolean fixOk0;
  Integer segNumBeforeSecondArc0;
  Real distDx0;
  Real distDy0;
  Real distToTurn0;

  linkIn = (GeoRev2CircleLinkInput_t){0};
  linkOut = (GeoRev2CircleLinkOutput_t){0};
  linkIn.Pt1X = stX;
  linkIn.Pt1Y = stY;
  linkIn.Pt1Theta = stTheta;
  linkIn.Pt2X = edX;
  linkIn.Pt2Y = edY;
  linkIn.Pt2Theta = edTheta;
  linkIn.Dir = linkDir;
  linkIn.R1 = radius;
  linkIn.R2 = radius;
  (void)pathType;
  geoRev2CircleLink(&linkIn, &linkOut);

  if (linkOut.GeoOk == 1) {
    geoVtPathPlanningMainLoopCheckCircle(input,
                                         param,
                                         work,
                                         linkOut.Ct1X,
                                         linkOut.Ct1Y,
                                         radius,
                                         stX,
                                         stY,
                                         stTheta,
                                         linkOut.St2X,
                                         linkOut.St2Y,
                                         linkOut.St2Theta,
                                         linkDir,
                                         linkOut.TrnD1,
                                         &arcChkOut);
    appendResult0 = geoVtPathPlanningMainLoopAppendArcFromCheck(work,
                                                                &arcChkOut,
                                                                -linkDir,
                                                                linkOut.TrnD1,
                                                                radius,
                                                                linkOut.Len1,
                                                                linkOut.St2X,
                                                                linkOut.St2Y,
                                                                linkOut.St2Theta,
                                                                linkOut.Ct1X,
                                                                linkOut.Ct1Y);
    if (appendResult0 == 1) {
      segNumBeforeSecondArc0 = work->SegNum;
      geoVtPathPlanningMainLoopCheckCircle(input,
                                           param,
                                           work,
                                           linkOut.Ct2X,
                                           linkOut.Ct2Y,
                                           radius,
                                           linkOut.St2X,
                                           linkOut.St2Y,
                                           linkOut.St2Theta,
                                           linkOut.St3X,
                                           linkOut.St3Y,
                                           linkOut.St3Theta,
                                           -linkDir,
                                           linkOut.TrnD2,
                                           &arcChkOut);
      fixTried0 = false;
      fixOk0 = false;
      if (arcChkOut.IntrvFlg == 0) {
        distDx0 = linkOut.St3X - edX;
        distDy0 = linkOut.St3Y - edY;
        distToTurn0 = (cosf(edTheta) * distDx0) + (sinf(edTheta) * distDy0);
        if (segNumBeforeSecondArc0 == 3) {
          fixTried0 = true;
        } else {
          if (segNumBeforeSecondArc0 == 2) {
            if (distToTurn0 < 0.0f) {
              fixTried0 = true;
            }
          }
        }
        if (fixTried0) {
          fixOk0 = geoVtPathPlanningMainLoopTryOneRFix(
              input, param, work, edX, edY, edTheta, radius, radius);
        }
      }
      if (!fixOk0) {
        appendResult0 = geoVtPathPlanningMainLoopAppendArcFromCheck(work,
                                                                    &arcChkOut,
                                                                    linkDir,
                                                                    linkOut.TrnD2,
                                                                    radius,
                                                                    linkOut.Len2,
                                                                    linkOut.St3X,
                                                                    linkOut.St3Y,
                                                                    linkOut.St3Theta,
                                                                    linkOut.Ct2X,
                                                                    linkOut.Ct2Y);
        if (appendResult0 == 1) {
          tailOk0 = geoVtPathPlanningMainLoopAppendTailLineIfNeeded(
              input, param, work, edX, edY, edTheta, linkOut.St3rX);
          if (tailOk0) {
            work->PlanCmd = 1;
          } else {
            geoVtPathPlanningMainLoopMarkFailure(work, 7);
          }
        }
      }
    }
  } else {
    geoVtPathPlanningMainLoopMarkFailure(work, geoFailReason);
  }
}

static void geoVtPathPlanningMainLoopRunLineCircle(const GeoVtPathPlanningInput_t *input,
                                                   const GeoVtPathPlanningRunParam_t *param,
                                                   GeoVtPathPlanningMainLoopWork_t *work,
                                                   Real stX,
                                                   Real stY,
                                                   Real stTheta,
                                                   Real edX,
                                                   Real edY,
                                                   Real edTheta,
                                                   Integer pathType,
                                                   Integer dirS,
                                                   Real radius) {
  GeoCircle2lineRInput_t circleIn;
  GeoCircle2lineROutput_t circleOut;
  GeoLineIntrvCheckOutput_t lineChkOut;
  GeoCircleIntrvCheckOutput_t arcChkOut;
  Real relX0;
  Real relY0;
  Real relTheta0;
  Integer lineGoDir0;
  Integer arcOutputGoDir0;
  Integer appendResult0;
  Boolean lineWriteOk0;
  Boolean tailOk0;

  circleIn = (GeoCircle2lineRInput_t){0};
  circleOut = (GeoCircle2lineROutput_t){0};
  circleIn.VstX = stX;
  circleIn.VstY = stY;
  circleIn.VstTheta = stTheta;
  circleIn.VtgtX = edX;
  circleIn.VtgtY = edY;
  circleIn.VtgtTheta = edTheta;
  circleIn.Ptyp = pathType;
  circleIn.Dir = dirS;
  circleIn.Radius = radius;
  geoCircle2lineR(&circleIn, &circleOut);

  geoVtPathPlanningMainLoopCheckLine(input,
                                     param,
                                     work,
                                     stX,
                                     stY,
                                     stTheta,
                                     circleOut.StPtX,
                                     circleOut.StPtY,
                                     circleOut.StPtTheta,
                                     GEO_VT_MAIN_LOOP_LINE_CHECK_STEP,
                                     VCC_SLOT_CLASS_VERTICAL,
                                     &lineChkOut);
  if (lineChkOut.IntrvFlg == 2) {
    geoVtPathPlanningMainLoopMarkFailure(work, 8);
  } else {
    if (lineChkOut.IntrvFlg == 1) {
      geoVtPathPlanningMainLoopMarkFailure(work, 7);
    } else {
      apaCoordGround2Veh(stX,
                         stY,
                         stTheta,
                         lineChkOut.AvlPosX,
                         lineChkOut.AvlPosY,
                         lineChkOut.AvlPosTheta,
                         &relX0,
                         &relY0,
                         &relTheta0);
      lineGoDir0 = 1;
      if (relX0 > 0.0f) {
        lineGoDir0 = -1;
      }
      lineWriteOk0 = geoVtPathPlanningMainLoopWriteLineToTail(
          work, lineGoDir0, circleOut.StPtX, circleOut.StPtY, circleOut.StPtTheta);
      if (lineWriteOk0) {
        geoVtPathPlanningMainLoopCheckCircle(input,
                                             param,
                                             work,
                                             circleOut.CptX,
                                             circleOut.CptY,
                                             radius,
                                             circleOut.StPtX,
                                             circleOut.StPtY,
                                             circleOut.StPtTheta,
                                             circleOut.EdPtX,
                                             circleOut.EdPtY,
                                             circleOut.EdPtTheta,
                                             dirS,
                                             circleOut.TrnDir,
                                             &arcChkOut);

        arcOutputGoDir0 = -dirS;
        if (arcChkOut.IntrvFlg == 0) {
          arcOutputGoDir0 = -circleOut.GoDir;
        }
        appendResult0 = geoVtPathPlanningMainLoopAppendArcFromCheck(work,
                                                                    &arcChkOut,
                                                                    arcOutputGoDir0,
                                                                    circleOut.TrnDir,
                                                                    radius,
                                                                    circleOut.ArcLen,
                                                                    circleOut.EdPtX,
                                                                    circleOut.EdPtY,
                                                                    circleOut.EdPtTheta,
                                                                    circleOut.CptX,
                                                                    circleOut.CptY);
        if (appendResult0 == 1) {
          tailOk0 = geoVtPathPlanningMainLoopAppendTailLineIfNeeded(
              input, param, work, edX, edY, edTheta, circleOut.EdPtrX);
          if (tailOk0) {
            work->PlanCmd = 1;
          } else {
            geoVtPathPlanningMainLoopMarkFailure(work, 7);
          }
        }
      } else {
        geoVtPathPlanningMainLoopMarkFailure(work, 7);
      }
    }
  }
}

static void geoVtPathPlanningMainLoopRunRevDistanceProbe(const GeoVtPathPlanningInput_t *input,
                                                         const GeoVtPathPlanningRunParam_t *param,
                                                         GeoVtPathPlanningMainLoopWork_t *work,
                                                         Real stX,
                                                         Real stY,
                                                         Real stTheta,
                                                         Real relX) {
  GeoLineIntrvCheckOutput_t lineChkOut;
  Real edX0;
  Real edY0;
  Real edTheta0;
  Real acceptedLen0;
  Integer tryDir0;
  Integer nextDirS0;
  Boolean acceptBackwardPartial0;
  Boolean writeOk0;
  Boolean tryForward0;
  Boolean needBackward0;

  tryForward0 = (relX > 0.0f);
  needBackward0 = true;
  if (tryForward0) {
    tryDir0 = 1;
    edX0 = stX + (GEO_VT_MAIN_LOOP_REV_DIST * (Real)tryDir0 * cosf(stTheta));
    edY0 = stY + (GEO_VT_MAIN_LOOP_REV_DIST * (Real)tryDir0 * sinf(stTheta));
    edTheta0 = stTheta;
    geoVtPathPlanningMainLoopCheckLine(input,
                                       param,
                                       work,
                                       stX,
                                       stY,
                                       stTheta,
                                       edX0,
                                       edY0,
                                       edTheta0,
                                       GEO_VT_MAIN_LOOP_REV_LINE_CHECK_STEP,
                                       VCC_SLOT_CLASS_PARALLEL,
                                       &lineChkOut);
    if (lineChkOut.IntrvFlg == 2) {
      needBackward0 = true;
    } else {
      if (lineChkOut.IntrvFlg == 0) {
        acceptedLen0 = GEO_VT_MAIN_LOOP_REV_DIST;
        writeOk0 = geoVtPathPlanningMainLoopWriteReverseLineSegment(
            work, -tryDir0, acceptedLen0, edX0, edY0, edTheta0);
        if (writeOk0) {
          work->DirS = 1;
          needBackward0 = false;
        } else {
          geoVtPathPlanningMainLoopMarkFailure(work, 7);
          needBackward0 = false;
        }
      } else {
        if (lineChkOut.SafeLen > (GEO_VT_MAIN_LOOP_REV_DIST * 0.5f)) {
          writeOk0 = geoVtPathPlanningMainLoopWriteReverseLineSegment(work,
                                                                      -tryDir0,
                                                                      lineChkOut.SafeLen,
                                                                      lineChkOut.AvlPosX,
                                                                      lineChkOut.AvlPosY,
                                                                      lineChkOut.AvlPosTheta);
          if (writeOk0) {
            work->DirS = -1;
          } else {
            geoVtPathPlanningMainLoopMarkFailure(work, 7);
          }
          needBackward0 = false;
        }
      }
    }
  }

  if (needBackward0) {
    tryDir0 = -1;
    edX0 = stX + (GEO_VT_MAIN_LOOP_REV_DIST * (Real)tryDir0 * cosf(stTheta));
    edY0 = stY + (GEO_VT_MAIN_LOOP_REV_DIST * (Real)tryDir0 * sinf(stTheta));
    edTheta0 = stTheta;
    geoVtPathPlanningMainLoopCheckLine(input,
                                       param,
                                       work,
                                       stX,
                                       stY,
                                       stTheta,
                                       edX0,
                                       edY0,
                                       edTheta0,
                                       GEO_VT_MAIN_LOOP_REV_LINE_CHECK_STEP,
                                       VCC_SLOT_CLASS_PARALLEL,
                                       &lineChkOut);
    if (lineChkOut.IntrvFlg == 2) {
      if (tryForward0) {
        geoVtPathPlanningMainLoopMarkFailure(work, 8);
      } else {
        work->DirS = 1;
      }
    } else {
      nextDirS0 = 1;
      if (lineChkOut.IntrvFlg == 0) {
        acceptedLen0 = GEO_VT_MAIN_LOOP_REV_DIST;
        nextDirS0 = -1;
        writeOk0 = geoVtPathPlanningMainLoopWriteReverseLineSegment(
            work, -tryDir0, acceptedLen0, edX0, edY0, edTheta0);
        if (writeOk0) {
          work->DirS = nextDirS0;
        } else {
          geoVtPathPlanningMainLoopMarkFailure(work, 7);
        }
      } else {
        acceptBackwardPartial0 = (lineChkOut.SafeLen > (GEO_VT_MAIN_LOOP_REV_DIST * 0.5f));
        if (tryForward0) {
          acceptBackwardPartial0 = true;
        }
        if (acceptBackwardPartial0) {
          writeOk0 = geoVtPathPlanningMainLoopWriteReverseLineSegment(work,
                                                                      -tryDir0,
                                                                      lineChkOut.SafeLen,
                                                                      lineChkOut.AvlPosX,
                                                                      lineChkOut.AvlPosY,
                                                                      lineChkOut.AvlPosTheta);
          if (writeOk0) {
            work->DirS = nextDirS0;
          } else {
            geoVtPathPlanningMainLoopMarkFailure(work, 7);
          }
        } else {
          tryDir0 = 1;
          edX0 = stX + (GEO_VT_MAIN_LOOP_REV_DIST * (Real)tryDir0 * cosf(stTheta));
          edY0 = stY + (GEO_VT_MAIN_LOOP_REV_DIST * (Real)tryDir0 * sinf(stTheta));
          edTheta0 = stTheta;
          geoVtPathPlanningMainLoopCheckLine(input,
                                             param,
                                             work,
                                             stX,
                                             stY,
                                             stTheta,
                                             edX0,
                                             edY0,
                                             edTheta0,
                                             GEO_VT_MAIN_LOOP_REV_LINE_CHECK_STEP,
                                             VCC_SLOT_CLASS_PARALLEL,
                                             &lineChkOut);
          if (lineChkOut.IntrvFlg == 2) {
            geoVtPathPlanningMainLoopMarkFailure(work, 8);
          } else {
            nextDirS0 = 1;
            if (lineChkOut.IntrvFlg == 0) {
              acceptedLen0 = GEO_VT_MAIN_LOOP_REV_DIST;
              writeOk0 = geoVtPathPlanningMainLoopWriteReverseLineSegment(
                  work, -tryDir0, acceptedLen0, edX0, edY0, edTheta0);
            } else {
              nextDirS0 = -1;
              writeOk0 = geoVtPathPlanningMainLoopWriteReverseLineSegment(work,
                                                                          -tryDir0,
                                                                          lineChkOut.SafeLen,
                                                                          lineChkOut.AvlPosX,
                                                                          lineChkOut.AvlPosY,
                                                                          lineChkOut.AvlPosTheta);
            }
            if (writeOk0) {
              work->DirS = nextDirS0;
            } else {
              geoVtPathPlanningMainLoopMarkFailure(work, 7);
            }
          }
        }
      }
    }
  }
}

static void geoVtPathPlanningMainLoopRunEvenBranch(const GeoVtPathPlanningInput_t *input,
                                                   const GeoVtPathPlanningRunParam_t *param,
                                                   GeoVtPathPlanningMainLoopWork_t *work,
                                                   Real stX,
                                                   Real stY,
                                                   Real stTheta,
                                                   Real edX,
                                                   Real edY,
                                                   Real edTheta,
                                                   Integer pathType) {
  GeoCircle1pt1lineInput_t circleIn;
  GeoCircle1pt1lineOutput_t circleOut;
  Real radius0;
  Real radiusDiff0;
  Real relX0;
  Real relY0;
  Real relTheta0;

  circleIn = (GeoCircle1pt1lineInput_t){0};
  circleOut = (GeoCircle1pt1lineOutput_t){0};
  circleIn.VstX = stX;
  circleIn.VstY = stY;
  circleIn.VstTheta = stTheta;
  circleIn.VtgtX = edX;
  circleIn.VtgtY = edY;
  circleIn.VtgtTheta = edTheta;
  circleIn.Dir = -work->DirS;
  geoCircle1pt1line(&circleIn, &circleOut);

  radius0 = fabsf(circleOut.Radius);
  radiusDiff0 = fabsf(radius0 - input->Rmin);
  geoVtPathPlanningMainLoopCalcRelativePose(
      edX, edY, edTheta, stX, stY, stTheta, &relX0, &relY0, &relTheta0);
  (void)relY0;
  (void)relTheta0;

  if (radiusDiff0 < GEO_VT_MAIN_LOOP_RADIUS_MATCH_EPS) {
    if ((relX0 * (Real)work->DirS) > 0.0f) {
      geoVtPathPlanningMainLoopRunCircleLine(
          input, param, work, &circleOut, work->DirS, -work->DirS, edX, edY, edTheta);
    } else {
      geoVtPathPlanningMainLoopRunRevDistanceProbe(input, param, work, stX, stY, stTheta, relX0);
    }
  } else {
    if (radius0 > input->Rmin) {
      geoVtPathPlanningMainLoopRunTwoCircleLink(
          input, param, work, stX, stY, stTheta, edX, edY, edTheta, pathType, work->DirS, 1.0f, 7);
    } else {
      geoVtPathPlanningMainLoopRunReverseTwoCircle(input,
                                                   param,
                                                   work,
                                                   stX,
                                                   stY,
                                                   stTheta,
                                                   edX,
                                                   edY,
                                                   edTheta,
                                                   pathType,
                                                   work->DirS,
                                                   input->Rmin,
                                                   0);
    }
  }
}

static void geoVtPathPlanningMainLoopRunOddBranch(const GeoVtPathPlanningInput_t *input,
                                                  const GeoVtPathPlanningRunParam_t *param,
                                                  GeoVtPathPlanningMainLoopWork_t *work,
                                                  Real stX,
                                                  Real stY,
                                                  Real stTheta,
                                                  Real edX,
                                                  Real edY,
                                                  Real edTheta,
                                                  Integer pathType,
                                                  Real stRelY,
                                                  Real stRelTheta) {
  GeoCircle1pt1lineInput_t circleIn;
  GeoCircle1pt1lineOutput_t circleOut;
  Real radius0;
  Integer subType0;
  Boolean tooLong0;
  Boolean postureSmall0;
  Boolean degradeToLineCircle0;

  circleIn = (GeoCircle1pt1lineInput_t){0};
  circleOut = (GeoCircle1pt1lineOutput_t){0};
  circleIn.VstX = stX;
  circleIn.VstY = stY;
  circleIn.VstTheta = stTheta;
  circleIn.VtgtX = edX;
  circleIn.VtgtY = edY;
  circleIn.VtgtTheta = edTheta;
  circleIn.Dir = work->DirS;
  geoCircle1pt1line(&circleIn, &circleOut);

  radius0 = fabsf(circleOut.Radius);
  if (radius0 >= input->Rmin) {
    tooLong0 = (circleOut.ArcLen > GEO_VT_MAIN_LOOP_CIR_LEN_MAX);
    if (fabsf(circleOut.LnPtRX) > GEO_VT_MAIN_LOOP_CIR_EDX_MAX) {
      tooLong0 = true;
    }
    postureSmall0 = false;
    if (fabsf(stRelY) < GEO_VT_MAIN_LOOP_CIR_EDY_MAX) {
      if (fabsf(stRelTheta) < (APA_PI / 12.0f)) {
        postureSmall0 = true;
      }
    }
    subType0 = 1;
    if (tooLong0) {
      if (postureSmall0) {
        subType0 = 2;
      } else {
        subType0 = 3;
      }
    }
    degradeToLineCircle0 = false;
    if (subType0 == 2) {
      geoVtPathPlanningMainLoopRunTwoCircleLink(
          input, param, work, stX, stY, stTheta, edX, edY, edTheta, pathType, work->DirS, 1.2f, 0);
      if (work->PlanCmd == 5) {
        if (work->FailRsOut[0] == 0) {
          work->PlanCmd = 0;
          degradeToLineCircle0 = true;
        }
      }
    }
    if (subType0 == 1) {
      geoVtPathPlanningMainLoopRunCircleLine(
          input, param, work, &circleOut, work->DirS, -work->DirS, edX, edY, edTheta);
    }
    if (subType0 == 3) {
      degradeToLineCircle0 = true;
    }
    if (degradeToLineCircle0) {
      geoVtPathPlanningMainLoopRunLineCircle(input,
                                             param,
                                             work,
                                             stX,
                                             stY,
                                             stTheta,
                                             edX,
                                             edY,
                                             edTheta,
                                             pathType,
                                             work->DirS,
                                             input->Rmin);
    }
  } else {
    geoVtPathPlanningMainLoopRunReverseTwoCircle(input,
                                                 param,
                                                 work,
                                                 stX,
                                                 stY,
                                                 stTheta,
                                                 edX,
                                                 edY,
                                                 edTheta,
                                                 pathType,
                                                 work->DirS,
                                                 input->Rmin + 0.2f,
                                                 7);
  }
}

static Boolean geoVtPathPlanningMainLoopRunPlanLoop(const GeoVtPathPlanningInput_t *input,
                                                    const GeoVtPathPlanningRunParam_t *param,
                                                    GeoVtPathPlanningMainLoopWork_t *work,
                                                    GeoVtPathPlanningOutput_t *output) {
  GeoPosTypClassPkInInput_t posIn;
  GeoPosTypClassPkInOutput_t posOut;
  Integer stageIndex0;
  Integer maxSeg0;
  Boolean loopActive0;
  Boolean planOk0;
  Boolean canIter0;
  Boolean isEven0;
  Boolean isOdd0;

  *output = (GeoVtPathPlanningOutput_t){0};
  work->PlanCmd = 0;
  work->DirS = 1;
  work->Calcnt = 0;
  maxSeg0 = 9 - work->IniSegN;
  loopActive0 = true;
  while (loopActive0) {
    canIter0 = (work->SegNum < maxSeg0);
    if (work->PlanCmd == 5) {
      canIter0 = false;
    }
    if (!canIter0) {
      loopActive0 = false;
    } else {
      stageIndex0 = geoVtPathPlanningMainLoopTailHeadIndex(work);
      posIn = (GeoPosTypClassPkInInput_t){0};
      posOut = (GeoPosTypClassPkInOutput_t){0};
      posIn.StPtX = work->Stx[stageIndex0];
      posIn.StPtY = work->Sty[stageIndex0];
      posIn.StPtTheta = work->StTh[stageIndex0];
      posIn.EdPtX = work->PlnEdPtX;
      posIn.EdPtY = work->PlnEdPtY;
      posIn.EdPtTheta = work->PlnEdPtTheta;
      posIn.Dir = work->DirS;
      geoPosTypClassPkIn(&posIn, &posOut);
      if (work->StpPthTypLN < GEO_VT_MAIN_LOOP_STAGE_MAX) {
        work->StpPthTypList[work->StpPthTypLN] = posOut.TarPTyp;
        work->StpPthTypLN = work->StpPthTypLN + 1;
      } else {
        work->StpPthTypList[GEO_VT_MAIN_LOOP_STAGE_MAX - 1] = posOut.TarPTyp;
        work->StpPthTypLN = GEO_VT_MAIN_LOOP_STAGE_MAX;
      }

      isEven0 = geoVtPathPlanningMainLoopIsEvenPathType(posOut.TarPTyp);
      isOdd0 = geoVtPathPlanningMainLoopIsOddPathType(posOut.TarPTyp);
      if (isEven0) {
        geoVtPathPlanningMainLoopRunEvenBranch(input,
                                               param,
                                               work,
                                               posIn.StPtX,
                                               posIn.StPtY,
                                               posIn.StPtTheta,
                                               posIn.EdPtX,
                                               posIn.EdPtY,
                                               posIn.EdPtTheta,
                                               posOut.TarPTyp);
      } else {
        if (isOdd0) {
          geoVtPathPlanningMainLoopRunOddBranch(input,
                                                param,
                                                work,
                                                posIn.StPtX,
                                                posIn.StPtY,
                                                posIn.StPtTheta,
                                                posIn.EdPtX,
                                                posIn.EdPtY,
                                                posIn.EdPtTheta,
                                                posOut.TarPTyp,
                                                posOut.PtRelaY,
                                                posOut.PtRelaTheta);
        } else {
          geoVtPathPlanningMainLoopMarkFailure(work, 0);
        }
      }

      if (work->PlanCmd == 5) {
        loopActive0 = false;
      } else {
        if (work->PlanCmd == 1) {
          loopActive0 = false;
        } else {
          if (work->PlanCmd == 2) {
            stageIndex0 = geoVtPathPlanningMainLoopTailHeadIndex(work);
            work->DirS = work->GoDir[stageIndex0];
            work->PlanCmd = 0;
          }
          work->Calcnt = work->Calcnt + 1;
          if (work->Calcnt > GEO_VT_MAIN_LOOP_ITER_MAX) {
            geoVtPathPlanningMainLoopMarkFailure(work, 9);
            loopActive0 = false;
          }
        }
      }
    }
  }

  planOk0 = false;
  if (work->PlanCmd == 1) {
    planOk0 = geoVtPathPlanningMainLoopAssembleOutput(work, output);
  } else {
    geoVtPathPlanningMainLoopLoadDiagToOutput(work, output);
  }

  return planOk0;
}

static void geoVtPathPlanningCalcSegEnd(Integer goDir,
                                        Integer trnDir,
                                        Real crv,
                                        Real tht,
                                        Real len,
                                        Real stX,
                                        Real stY,
                                        Real stTheta,
                                        Real ccX,
                                        Real ccY,
                                        Real *endX,
                                        Real *endY,
                                        Real *endTheta) {
  if ((trnDir != 0) && (fabsf(crv) > GEO_EPSILON)) {
    Real invCrv;
    Real theta;

    invCrv = 1.0f / crv;
    theta = stTheta + ((Real)goDir) * tht;
    *endX = ccX + invCrv * sinf(theta);
    *endY = ccY - invCrv * cosf(theta);
    *endTheta = theta;
  } else {
    *endX = stX + ((Real)goDir) * len * cosf(stTheta);
    *endY = stY + ((Real)goDir) * len * sinf(stTheta);
    *endTheta = stTheta + tht;
  }
}

static void geoVtPathPlanningHybridEvalPrimitivePose(Integer goDir,
                                                     Real curv,
                                                     Real travel,
                                                     Real stX,
                                                     Real stY,
                                                     Real stTheta,
                                                     Real *poseX,
                                                     Real *poseY,
                                                     Real *poseTheta) {
  if (fabsf(curv) <= GEO_VT_HYBRID_CURV_EPS) {
    *poseX = stX + (Real)goDir * travel * cosf(stTheta);
    *poseY = stY + (Real)goDir * travel * sinf(stTheta);
    *poseTheta = stTheta;
  } else {
    Real signedTravel;
    Real invCurv;
    Real centerX;
    Real centerY;
    Real theta;

    signedTravel = (Real)goDir * travel;
    invCurv = 1.0f / curv;
    centerX = stX - invCurv * sinf(stTheta);
    centerY = stY + invCurv * cosf(stTheta);
    theta = apaCoordWrapToPi(stTheta + curv * signedTravel);
    *poseX = centerX + invCurv * sinf(theta);
    *poseY = centerY - invCurv * cosf(theta);
    *poseTheta = theta;
  }
}

/**
 * @brief Check the finite fields and the arc metadata used by discretization.
 * @en_name geoVtPathPlanningSegmentGeometryIsValid
 * @cn_name VT几何段字段检查
 * @type module
 * @param[IN] GeoVtPathPlanningOutput_t Candidate 分段候选
 * @param[IN] Integer Index 段0-based索引
 * @param[OUT] Boolean Valid 返回有效标志
 * @retval Boolean 字段及圆弧几何是否一致
 * @granularity atomic
 * @tag_level1 规划
 * @source safety_only
 * @version 1.0
 * @date 2026-09-30
 * @author aide
 */
static Boolean geoVtPathPlanningSegmentGeometryIsValid(const GeoVtPathPlanningOutput_t *candidate,
                                                       Integer index) {
  const Real fields[8] = {candidate->SegLen[index],
                          candidate->SegCrv[index],
                          candidate->SegTht[index],
                          candidate->SegStX[index],
                          candidate->SegStY[index],
                          candidate->SegStTh[index],
                          candidate->SegCcX[index],
                          candidate->SegCcY[index]};
  Integer fieldIndex;
  Boolean valid0;
  valid0 = (candidate->SegLen[index] >= 0.0f);
  if (valid0) {
    valid0 = (candidate->SegGoDir[index] == 1) || (candidate->SegGoDir[index] == -1);
  }
  fieldIndex = 0;
  while ((fieldIndex < 8) && valid0) {
    valid0 = isfinite(fields[fieldIndex]);
    fieldIndex = fieldIndex + 1;
  }
  if (valid0) {
    if (fabsf(candidate->SegCrv[index]) > 1.0e-6f) {
      Real dx0;
      Real dy0;
      Real centerError0;
      Real arcError0;
      dx0 = candidate->SegStX[index] -
            (sinf(candidate->SegStTh[index]) / candidate->SegCrv[index]) - candidate->SegCcX[index];
      dy0 = candidate->SegStY[index] +
            (cosf(candidate->SegStTh[index]) / candidate->SegCrv[index]) - candidate->SegCcY[index];
      centerError0 = sqrtf((dx0 * dx0) + (dy0 * dy0));

      arcError0 = fabsf(candidate->SegCrv[index] * candidate->SegLen[index]) -
                  fabsf(candidate->SegTht[index]);
      valid0 = (centerError0 <= 0.02f) && (arcError0 <= 0.003f) &&
               ((candidate->SegCrv[index] * ((Real)candidate->SegTrnDir[index])) > 0.0f);
    } else {
      valid0 = (candidate->SegTrnDir[index] == 0) && (fabsf(candidate->SegTht[index]) <= 0.003f);
    }
  }
  return valid0;
}

/**
 * @brief Verify constant-curvature segments against their next pose and target.
 * @en_name geoVtPathPlanningGeometryIsContinuous
 * @cn_name VT几何段一致性检查
 * @type module
 * @param[IN] GeoVtPathPlanningInput_t Input 目标位姿
 * @param[IN] GeoVtPathPlanningOutput_t Candidate 分段候选
 * @param[OUT] Integer RejectedSegment 失败段1-based无失败为0
 * @retval Boolean 是否连续且满足段运动学
 * @granularity atomic
 * @tag_level1 规划
 * @source safety_only
 * @version 1.0
 * @date 2026-09-30
 * @author aide
 */
Boolean geoVtPathPlanningGeometryIsContinuous(const GeoVtPathPlanningInput_t *input,
                                              const GeoVtPathPlanningOutput_t *candidate,
                                              Integer *rejectedSegment) {
  Integer index;
  Boolean valid0;
  Boolean loop0;
  *rejectedSegment = 0;
  valid0 = (candidate->SegNum > 0);
  if (valid0) {
    valid0 = (candidate->SegNum <= VT_SEG_OUT_MAX);
  }
  index = 0;
  loop0 = valid0;
  while (loop0) {
    Real endX;
    Real endY;
    Real endTheta;
    Real targetX0;
    Real targetY0;
    Real targetTheta0;
    Real dx0;
    Real dy0;
    Real distance0;
    Real angle0;
    valid0 = geoVtPathPlanningSegmentGeometryIsValid(candidate, index);
    if (valid0) {
      geoVtPathPlanningHybridEvalPrimitivePose(candidate->SegGoDir[index],
                                               candidate->SegCrv[index],
                                               candidate->SegLen[index],
                                               candidate->SegStX[index],
                                               candidate->SegStY[index],
                                               candidate->SegStTh[index],
                                               &endX,
                                               &endY,
                                               &endTheta);
      targetX0 = input->TarX;
      targetY0 = input->TarY;
      targetTheta0 = input->TarTheta;
      if ((index + 1) < candidate->SegNum) {
        targetX0 = candidate->SegStX[index + 1];
        targetY0 = candidate->SegStY[index + 1];
        targetTheta0 = candidate->SegStTh[index + 1];
      }
      dx0 = endX - targetX0;
      dy0 = endY - targetY0;
      distance0 = sqrtf((dx0 * dx0) + (dy0 * dy0));
      angle0 = apaCoordWrapToPi(endTheta - targetTheta0);
      valid0 = isfinite(distance0);
      if (valid0) {
        valid0 = isfinite(angle0);
      }
      if (valid0) {
        valid0 = (distance0 <= GEO_VT_HYBRID_FIT_POS_EPS);
      }
      if (valid0) {
        valid0 = (fabsf(angle0) <= GEO_VT_HYBRID_FIT_YAW_EPS);
      }
    }
    if (!valid0) {
      *rejectedSegment = index + 1;
    }
    index = index + 1;
    loop0 = (index < candidate->SegNum);
    if (!valid0) {
      loop0 = false;
    }
  }
  return valid0;
}

static Boolean geoVtPathPlanningIsCandidateSafe(const GeoVtPathPlanningInput_t *input,
                                                const GeoVtPathPlanningRunParam_t *param,
                                                const GeoVtPathPlanningOutput_t *candidate,
                                                Integer *plnChkCnt) {
  Integer pathSegNum;
  Integer pathGoDir[APA_PATH_SEG_MAX];
  Integer pathTrnDir[APA_PATH_SEG_MAX];
  Real pathCrv[APA_PATH_SEG_MAX];
  Real pathTht[APA_PATH_SEG_MAX];
  Real pathLen[APA_PATH_SEG_MAX];
  Real pathStx[APA_PATH_SEG_MAX];
  Real pathSty[APA_PATH_SEG_MAX];
  Real pathStTh[APA_PATH_SEG_MAX];
  Real pathCC[APA_PATH_CC_MAX];
  Integer idx;
  Boolean safeOk;

  geoVtPathPlanningExportCandidate(candidate,
                                   &pathSegNum,
                                   pathGoDir,
                                   pathTrnDir,
                                   pathCrv,
                                   pathTht,
                                   pathLen,
                                   pathStx,
                                   pathSty,
                                   pathStTh,
                                   pathCC);
  safeOk = (pathSegNum > 0);
  idx = 0;
  while ((idx < pathSegNum) && safeOk) {
    Real endX;
    Real endY;
    Real endTheta;

    if ((idx + 1) < pathSegNum) {
      if (pathGoDir[idx] == pathGoDir[idx + 1]) {
        endX = pathStx[idx + 1];
        endY = pathSty[idx + 1];
        endTheta = pathStTh[idx + 1];
      } else {
        geoVtPathPlanningCalcSegEnd(pathGoDir[idx],
                                    pathTrnDir[idx],
                                    pathCrv[idx],
                                    pathTht[idx],
                                    pathLen[idx],
                                    pathStx[idx],
                                    pathSty[idx],
                                    pathStTh[idx],
                                    pathCC[idx * 2],
                                    pathCC[idx * 2 + 1],
                                    &endX,
                                    &endY,
                                    &endTheta);
      }
    } else {
      if (pathTrnDir[idx] == 0) {
        endX = input->TarX;
        endY = input->TarY;
        endTheta = input->TarTheta;
      } else {
        geoVtPathPlanningCalcSegEnd(pathGoDir[idx],
                                    pathTrnDir[idx],
                                    pathCrv[idx],
                                    pathTht[idx],
                                    pathLen[idx],
                                    pathStx[idx],
                                    pathSty[idx],
                                    pathStTh[idx],
                                    pathCC[idx * 2],
                                    pathCC[idx * 2 + 1],
                                    &endX,
                                    &endY,
                                    &endTheta);
      }
    }

    if ((pathTrnDir[idx] != 0) && (fabsf(pathCrv[idx]) > GEO_EPSILON)) {
      GeoCircleIntrvCheckInput_t chkIn;
      GeoCircleIntrvCheckOutput_t chkOut;
      Real radius;

      radius = fabsf(1.0f / pathCrv[idx]);
      geoVtPathPlanningFillCircleCheck(input,
                                       param,
                                       pathCC[idx * 2],
                                       pathCC[idx * 2 + 1],
                                       radius,
                                       pathStx[idx],
                                       pathSty[idx],
                                       pathStTh[idx],
                                       endX,
                                       endY,
                                       endTheta,
                                       pathGoDir[idx],
                                       pathTrnDir[idx],
                                       GEO_VT_FRONT_ARC_CHECK_STEP,
                                       &chkIn);
      chkOut = (GeoCircleIntrvCheckOutput_t){0};
      geoVtPathPlanningCheckCircleWithPolicy(&chkIn, &chkOut, param);
      *plnChkCnt = *plnChkCnt + chkOut.CheckCount;
      if (chkOut.IntrvFlg > 0) {
        safeOk = false;
      }
    } else {
      GeoLineIntrvCheckInput_t chkIn;
      GeoLineIntrvCheckOutput_t chkOut;

      geoVtPathPlanningFillLineCheck(input,
                                     param,
                                     pathStx[idx],
                                     pathSty[idx],
                                     pathStTh[idx],
                                     endX,
                                     endY,
                                     endTheta,
                                     GEO_VT_FRONT_DIRECT_LINE_STEP,
                                     VCC_SLOT_CLASS_VERTICAL,
                                     &chkIn);
      chkOut = (GeoLineIntrvCheckOutput_t){0};
      geoVtPathPlanningCheckLineWithPolicy(&chkIn, &chkOut, param);
      *plnChkCnt = *plnChkCnt + chkOut.CheckCount;
      if (chkOut.IntrvFlg > 0) {
        safeOk = false;
      }
    }
    idx = idx + 1;
  }

  if (safeOk) {
    safeOk = geoVtPathPlanningAcceptPolicyCandidate(param, candidate);
  }
  return safeOk;
}

static Boolean geoVtPathPlanningHasValidCandidate(const GeoVtPathPlanningOutput_t *candidate) {
  Boolean hasPlanFlg;
  Boolean hasSegNum;
  Boolean hasPathLen;
  Boolean validCandidate0;

  hasPlanFlg = (candidate->PlanFlg == 1);
  hasSegNum = (candidate->SegNum > 0);
  hasPathLen = (candidate->PathLen > GEO_EPSILON);
  validCandidate0 = hasPlanFlg && hasSegNum && hasPathLen;

  return validCandidate0;
}

static Boolean
geoVtPathPlanningCandidateIsHybridFallback(const GeoVtPathPlanningOutput_t *candidate) {
  Boolean isHybridFallback0;

  isHybridFallback0 = (candidate->PlannerCase == GEO_VT_PLANNER_CASE_HYBRID_FALLBACK);

  return isHybridFallback0;
}

static Boolean geoVtPathPlanningHybridHasSignificantPathAdvantage(
    const GeoVtPathPlanningOutput_t *hybridCandidate,
    const GeoVtPathPlanningOutput_t *otherCandidate) {
  Integer stepDelta;
  Real pathGain;
  Real pathGainRatio;
  Boolean positiveStepDelta;
  Boolean withinStepBudget;
  Boolean enoughPathGain;
  Boolean enoughPathRatio;
  Boolean significantAdvantage0;

  stepDelta = hybridCandidate->StpN - otherCandidate->StpN;
  pathGain = otherCandidate->PathLen - hybridCandidate->PathLen;
  pathGainRatio = 0.0f;
  if (otherCandidate->PathLen > GEO_EPSILON) {
    pathGainRatio = pathGain / otherCandidate->PathLen;
  }
  positiveStepDelta = (stepDelta > 0);
  withinStepBudget = positiveStepDelta && (stepDelta <= GEO_VT_HYBRID_OVERRIDE_STEP_DELTA_MAX);
  enoughPathGain = (pathGain > GEO_VT_HYBRID_OVERRIDE_PATH_GAIN_MIN);
  enoughPathRatio = (pathGainRatio > GEO_VT_HYBRID_OVERRIDE_PATH_RATIO_MIN);
  significantAdvantage0 = false;
  if (withinStepBudget && enoughPathGain && enoughPathRatio) {
    significantAdvantage0 = true;
  }

  return significantAdvantage0;
}

static Integer
geoVtPathPlanningResolveCandidateDecisionReason(const GeoVtPathPlanningOutput_t *currentCandidate,
                                                const GeoVtPathPlanningOutput_t *nextCandidate,
                                                Boolean *replaceCandidate) {
  Boolean currentValid;
  Boolean nextValid;
  Boolean currentHybrid;
  Boolean nextHybrid;
  Boolean preferHybridPathQuality;
  Boolean shorterStepCount;
  Boolean longerStepCount;
  Boolean shorterPath;
  Boolean longerPath;
  Boolean fewerSegments;
  Boolean moreSegments;
  Boolean fewerChecks;
  Boolean moreChecks;
  Integer decisionReason0;

  currentValid = geoVtPathPlanningHasValidCandidate(currentCandidate);
  nextValid = geoVtPathPlanningHasValidCandidate(nextCandidate);
  *replaceCandidate = false;
  decisionReason0 = GEO_VT_FALLBACK_DECISION_NONE;

  if (nextValid) {
    if (!currentValid) {
      *replaceCandidate = true;
      decisionReason0 = GEO_VT_FALLBACK_DECISION_VALID_ONLY;
    } else {
      currentHybrid = geoVtPathPlanningCandidateIsHybridFallback(currentCandidate);
      nextHybrid = geoVtPathPlanningCandidateIsHybridFallback(nextCandidate);
      shorterStepCount = (nextCandidate->StpN < currentCandidate->StpN);
      longerStepCount = (nextCandidate->StpN > currentCandidate->StpN);
      shorterPath =
          ((nextCandidate->PathLen + GEO_VT_CANDIDATE_PATH_EPS) < currentCandidate->PathLen);
      longerPath =
          ((currentCandidate->PathLen + GEO_VT_CANDIDATE_PATH_EPS) < nextCandidate->PathLen);
      fewerSegments = (nextCandidate->SegNum < currentCandidate->SegNum);
      moreSegments = (nextCandidate->SegNum > currentCandidate->SegNum);
      fewerChecks = (nextCandidate->PlnChkCnt < currentCandidate->PlnChkCnt);
      moreChecks = (nextCandidate->PlnChkCnt > currentCandidate->PlnChkCnt);
      preferHybridPathQuality = false;
      if ((!currentHybrid) && nextHybrid) {
        preferHybridPathQuality =
            geoVtPathPlanningHybridHasSignificantPathAdvantage(nextCandidate, currentCandidate);
      } else {
        if (currentHybrid && (!nextHybrid)) {
          if (geoVtPathPlanningHybridHasSignificantPathAdvantage(currentCandidate, nextCandidate)) {
            decisionReason0 = GEO_VT_FALLBACK_DECISION_PATH_LEN;
          }
        }
      }

      if (preferHybridPathQuality) {
        *replaceCandidate = true;
        decisionReason0 = GEO_VT_FALLBACK_DECISION_PATH_LEN;
      } else {
        if (decisionReason0 != GEO_VT_FALLBACK_DECISION_PATH_LEN) {
          if (shorterStepCount) {
            *replaceCandidate = true;
            decisionReason0 = GEO_VT_FALLBACK_DECISION_STEP_COUNT;
          } else {
            if (longerStepCount) {
              decisionReason0 = GEO_VT_FALLBACK_DECISION_STEP_COUNT;
            } else {
              if (shorterPath) {
                *replaceCandidate = true;
                decisionReason0 = GEO_VT_FALLBACK_DECISION_PATH_LEN;
              } else {
                if (longerPath) {
                  decisionReason0 = GEO_VT_FALLBACK_DECISION_PATH_LEN;
                } else {
                  if (fewerSegments) {
                    *replaceCandidate = true;
                    decisionReason0 = GEO_VT_FALLBACK_DECISION_SEG_NUM;
                  } else {
                    if (moreSegments) {
                      decisionReason0 = GEO_VT_FALLBACK_DECISION_SEG_NUM;
                    } else {
                      if (fewerChecks) {
                        *replaceCandidate = true;
                        decisionReason0 = GEO_VT_FALLBACK_DECISION_PLN_CHK_CNT;
                      } else {
                        if (moreChecks) {
                          decisionReason0 = GEO_VT_FALLBACK_DECISION_PLN_CHK_CNT;
                        }
                      }
                    }
                  }
                }
              }
            }
          }
        }
      }
    }
  }

  return decisionReason0;
}

static Boolean
geoVtPathPlanningShouldReplaceCandidate(const GeoVtPathPlanningOutput_t *currentCandidate,
                                        const GeoVtPathPlanningOutput_t *nextCandidate,
                                        const GeoVtPathPlanningRunParam_t *param) {
  if (param->Base.HybridPolicy.CompareCandidates && currentCandidate->PlanFlg &&
      nextCandidate->PlanFlg) {
    ApaPlannerHybridSegments_t first, second;
    if (apaPlannerHybridReadGeometry(
            APA_HYBRID_GEOMETRY_FIELDS(currentCandidate, VT_SEG_OUT_MAX), &first, 0) &&
        apaPlannerHybridReadGeometry(
            APA_HYBRID_GEOMETRY_FIELDS(nextCandidate, VT_SEG_OUT_MAX), &second, 0))
      return param->Base.HybridPolicy.CompareCandidates(
                 &first, &second, param->Base.HybridPolicy.UserData) ==
             APA_PATH_COMP_NEW_PATH_BETTER;
  }

  Boolean replaceCandidate;

  replaceCandidate = false;
  (void)geoVtPathPlanningResolveCandidateDecisionReason(
      currentCandidate, nextCandidate, &replaceCandidate);

  return replaceCandidate;
}

static void geoVtPathPlanningCommitCandidate(const GeoVtPathPlanningOutput_t *candidate,
                                             Integer priorChkCnt,
                                             GeoVtPathPlanningOutput_t *output) {
  *output = *candidate;
  output->PlnChkCnt = output->PlnChkCnt + priorChkCnt;
}

static void
geoVtPathPlanningStoreFallbackDiagFromCandidate(Boolean candidateOk,
                                                const GeoVtPathPlanningOutput_t *candidate,
                                                Integer *diagStpN,
                                                Real *diagPathLen,
                                                Integer *diagSegNum,
                                                Integer *diagPlnChkCnt) {
  if (candidateOk) {
    *diagStpN = candidate->StpN;
    *diagPathLen = candidate->PathLen;
    *diagSegNum = candidate->SegNum;
    *diagPlnChkCnt = candidate->PlnChkCnt;
  } else {
    *diagStpN = 0;
    *diagPathLen = 0.0f;
    *diagSegNum = 0;
    *diagPlnChkCnt = 0;
  }
}

static Integer geoVtPathPlanningResolveFallbackWinnerKind(
    Boolean geometricOk, Boolean hybridOk, const GeoVtPathPlanningOutput_t *bestCandidate) {
  Boolean bestValid;
  Boolean hybridWinner;
  Integer winnerKind0;

  bestValid = geoVtPathPlanningHasValidCandidate(bestCandidate);
  winnerKind0 = GEO_VT_FALLBACK_WINNER_NONE;
  if (bestValid) {
    hybridWinner =
        (hybridOk && (bestCandidate->PlannerCase == GEO_VT_PLANNER_CASE_HYBRID_FALLBACK));
    if (hybridWinner) {
      winnerKind0 = GEO_VT_FALLBACK_WINNER_HYBRID;
    } else {
      if (geometricOk) {
        winnerKind0 = GEO_VT_FALLBACK_WINNER_GEOM;
      }
    }
  }

  return winnerKind0;
}

static Integer
geoVtPathPlanningResolveFallbackDecisionReason(Boolean geometricOk,
                                               Boolean hybridOk,
                                               const GeoVtPathPlanningOutput_t *geometricCandidate,
                                               const GeoVtPathPlanningOutput_t *hybridCandidate,
                                               const GeoVtPathPlanningRunParam_t *param) {
  if (geometricOk && hybridOk && param->Base.HybridPolicy.CompareCandidates)
    return 6;

  Boolean replaceHybrid;
  Integer decisionReason0;

  replaceHybrid = false;
  decisionReason0 = GEO_VT_FALLBACK_DECISION_NONE;
  if (geometricOk && hybridOk) {
    decisionReason0 = geoVtPathPlanningResolveCandidateDecisionReason(
        geometricCandidate, hybridCandidate, &replaceHybrid);
  } else {
    if (geometricOk || hybridOk) {
      decisionReason0 = GEO_VT_FALLBACK_DECISION_VALID_ONLY;
    }
  }

  return decisionReason0;
}

static void
geoVtPathPlanningHybridAccumulateMapBound(Real x, Real y, Real *minCoord, Real *maxCoord) {
  if (x < *minCoord) {
    *minCoord = x;
  }
  if (y < *minCoord) {
    *minCoord = y;
  }
  if (x > *maxCoord) {
    *maxCoord = x;
  }
  if (y > *maxCoord) {
    *maxCoord = y;
  }
}

static void geoVtPathPlanningHybridExpandMapForInput(const GeoVtPathPlanningInput_t *input,
                                                     ApaPlannerGlobalState_t *plannerGlobal) {
  Real minCoord;
  Real maxCoord;
  Real defaultHalfEdge;
  Real requiredHalfEdge;
  Real maxAllowedHalfEdge;
  Real gridSpan;
  Real margin;
  Integer idx;

  minCoord = input->VehX;
  maxCoord = input->VehX;
  geoVtPathPlanningHybridAccumulateMapBound(input->VehX, input->VehY, &minCoord, &maxCoord);
  geoVtPathPlanningHybridAccumulateMapBound(input->TarX, input->TarY, &minCoord, &maxCoord);
  geoVtPathPlanningHybridAccumulateMapBound(input->SltPt1X, input->SltPt1Y, &minCoord, &maxCoord);
  geoVtPathPlanningHybridAccumulateMapBound(input->SltPt2X, input->SltPt2Y, &minCoord, &maxCoord);
  geoVtPathPlanningHybridAccumulateMapBound(input->SltPt3X, input->SltPt3Y, &minCoord, &maxCoord);
  geoVtPathPlanningHybridAccumulateMapBound(input->SltPt4X, input->SltPt4Y, &minCoord, &maxCoord);

  idx = 0;
  while ((idx < input->BoundaryLineNum) && (idx < GEO_BOUNDARY_LINE_MAX)) {
    geoVtPathPlanningHybridAccumulateMapBound(
        input->BoundaryLineX1[idx], input->BoundaryLineY1[idx], &minCoord, &maxCoord);
    geoVtPathPlanningHybridAccumulateMapBound(
        input->BoundaryLineX2[idx], input->BoundaryLineY2[idx], &minCoord, &maxCoord);
    idx = idx + 1;
  }

  margin = GEO_VT_HYBRID_MAP_MARGIN;
  defaultHalfEdge = 0.5f * plannerGlobal->Planner.Map.MapEdgeLen;
  requiredHalfEdge = defaultHalfEdge;
  if (minCoord < 0.0f) {
    requiredHalfEdge = -minCoord + margin;
  }
  if (requiredHalfEdge < defaultHalfEdge) {
    requiredHalfEdge = defaultHalfEdge;
  }

  gridSpan = plannerGlobal->Planner.Map.XyGridResolution *
             (Real)(plannerGlobal->Planner.Map.MapEdgePixNum - 1);
  maxAllowedHalfEdge = gridSpan - maxCoord - margin;
  if ((maxAllowedHalfEdge > defaultHalfEdge) && (requiredHalfEdge > maxAllowedHalfEdge)) {
    requiredHalfEdge = maxAllowedHalfEdge;
  }
  if (requiredHalfEdge > defaultHalfEdge) {
    plannerGlobal->Planner.Map.MapEdgeLen = 2.0f * requiredHalfEdge;
  }
}

static void geoVtPathPlanningHybridInitPlannerGlobal(const GeoVtPathPlanningInput_t *input,
                                                     const GeoVtPathPlanningRunParam_t *param,
                                                     ApaPlannerGlobalState_t *plannerGlobal) {
  Integer needHybridAlgFusion;

  apaPlannerGlobalInit(plannerGlobal);
  needHybridAlgFusion = apaPlannerHybridResolveFusion(param->Base.NeedHybridAlgFusion);
  geoVtPathPlanningHybridExpandMapForInput(input, plannerGlobal);
  plannerGlobal->Planner.Map.MotionResolution = GEO_VT_HYBRID_COLLISION_STEP;
  plannerGlobal->Planner.Map.CollisionDetectStepLength = GEO_VT_HYBRID_COLLISION_STEP;

  plannerGlobal->Planner.Veh.W = param->Base.VehW;
  plannerGlobal->Planner.Veh.LF = param->Base.VehLlF;
  plannerGlobal->Planner.Veh.LB = param->Base.VehLr;
  plannerGlobal->Planner.Veh.LL = plannerGlobal->Planner.Veh.LF + plannerGlobal->Planner.Veh.LB;
  plannerGlobal->Planner.Veh.Lf = plannerGlobal->Planner.Veh.LF - plannerGlobal->Planner.Veh.WB;
  plannerGlobal->Planner.Veh.MinCircle = input->Rmin;
  if (input->Rmin > GEO_EPSILON) {
    plannerGlobal->Planner.Veh.MaxSteer = atanf(plannerGlobal->Planner.Veh.WB / input->Rmin);
  }
  plannerGlobal->Planner.Conf.PlanningCalPowerMaxPerSlot = GEO_VT_HYBRID_MAX_ITER;
  plannerGlobal->Planner.Conf.LongAllowableErrMin = -GEO_VT_HYBRID_GOAL_LONG_ERR;
  plannerGlobal->Planner.Conf.LongAllowableErrMax = GEO_VT_HYBRID_GOAL_LONG_ERR;
  plannerGlobal->Planner.Conf.LatAllowableErrMin = -GEO_VT_HYBRID_GOAL_LAT_ERR;
  plannerGlobal->Planner.Conf.LatAllowableErrMax = GEO_VT_HYBRID_GOAL_LAT_ERR;
  plannerGlobal->Planner.Conf.YawAllowableErrMin = -GEO_VT_HYBRID_GOAL_YAW_ERR;
  plannerGlobal->Planner.Conf.YawAllowableErrMax = GEO_VT_HYBRID_GOAL_YAW_ERR;
  apaPlannerGlobalSetNeedHybridAlgFusion(plannerGlobal, needHybridAlgFusion);
}

typedef struct {
  ApaVehicleCollisionCheckContext_t CollisionContext;
  const ApaTargetSlotConstraint_t *SlotConstraint;
  ApaPlanningSceneQuery_t SceneQuery;
  VehicleCollisionCheckInput_t SceneTemplate;
  Integer CheckCount;
} GeoVtPathPlanningHybridCollisionContext_t;

static void geoVtPathPlanningHybridLoadObs(const GeoVtPathPlanningInput_t *input,
                                           ApaVehicleCollisionObs_t *obs) {
  Integer idx;

  idx = 0;
  while ((idx < input->BoundaryLineNum) && (idx < GEO_BOUNDARY_LINE_MAX)) {
    (void)apaVehicleCollisionObsAddBoundaryLine(obs,
                                                input->BoundaryLineX1[idx],
                                                input->BoundaryLineY1[idx],
                                                input->BoundaryLineX2[idx],
                                                input->BoundaryLineY2[idx]);
    idx = idx + 1;
  }

  idx = 0;
  while ((idx < input->VirtualBoundaryLineNum) && (idx < GEO_BOUNDARY_LINE_MAX)) {
    (void)apaVehicleCollisionObsAddVirtualBoundaryLine(obs,
                                                       input->VirtualBoundaryP1X[idx],
                                                       input->VirtualBoundaryP1Y[idx],
                                                       input->VirtualBoundaryP2X[idx],
                                                       input->VirtualBoundaryP2Y[idx]);
    idx = idx + 1;
  }

  idx = 0;
  while ((idx < input->BoundarySepPtNum) && (idx < GEO_BOUNDARY_POINT_MAX)) {
    (void)apaVehicleCollisionObsAddBoundarySeparatePoint(
        obs, input->BoundarySepPtX[idx], input->BoundarySepPtY[idx]);
    idx = idx + 1;
  }

  idx = 0;
  while ((idx < input->StopperPtNum) && (idx < GEO_BOUNDARY_POINT_MAX)) {
    (void)apaVehicleCollisionObsAddSeparateStopper(
        obs, input->StopperPtX[idx], input->StopperPtY[idx]);
    idx = idx + 1;
  }
}

static void
geoVtPathPlanningHybridInitCollisionContext(const GeoVtPathPlanningInput_t *input,
                                            const ApaSlot_t *slot,
                                            const GeoVtPathPlanningRunParam_t *param,
                                            ApaPlannerGlobalState_t *plannerGlobal,
                                            GeoVtPathPlanningHybridCollisionContext_t *context) {
  context->SlotConstraint = geoVtPathPlanningSlotConstraint(param);
  context->SceneQuery = param->Base.SceneQuery;
  geoVtPathPlanningBuildTargetPrecheckCollisionInput(
      input, param, input->VehX, input->VehY, input->VehTheta, &context->SceneTemplate);
  context->CheckCount = 0;
  apaVehicleCollisionCheckContextInit(&context->CollisionContext);
  apaVehicleCollisionCheckContextSetPlannerGlobal(&context->CollisionContext, plannerGlobal);
  apaVehicleCollisionCheckContextSetCollisionDemand(&context->CollisionContext,
                                                    APA_COLLISION_DEMAND_ALL);
  apaVehicleCollisionCheckContextSetSlot(&context->CollisionContext, slot);
  apaVehicleCollisionCheckContextSetTargetPose(&context->CollisionContext, slot->TarPos);
  geoVtPathPlanningHybridLoadObs(input, &context->CollisionContext.Obs);
}

/**
 * @brief Check a Hybrid primitive against the separate slot-frame policy.
 * @en_name geoVtPathPlanningHybridSlotCollision
 * @cn_name VT Hybrid车位约束检查
 * @type module
 * @param[IN] GeoVtPathPlanningHybridCollisionContext_t Context 本次碰撞上下文
 * @param[IN] ApaPlannerConfig_t Config 车辆与采样配置
 * @param[IN] ApaHybridNode_t FromNode 起始后轴节点
 * @param[IN] ApaHybridNode_t ToNode 运动终止节点
 * @param[OUT] Boolean Collision 返回相交标志
 * @retval Boolean 是否触及车位非入口边
 * @granularity composite
 * @tag_level1 规划
 * @source safety_only
 * @version 1.0
 * @date 2026-09-30
 * @author aide
 */
static Boolean
geoVtPathPlanningHybridSlotCollision(GeoVtPathPlanningHybridCollisionContext_t *context,
                                     const ApaPlannerConfig_t *config,
                                     const ApaHybridNode_t *fromNode,
                                     const ApaHybridNode_t *toNode) {
  ApaTargetSlotMotion_t motion;
  ApaTargetSlotMotionCheck_t frame;
  Boolean collision0;
  collision0 = false;
  if (context->SlotConstraint != (const ApaTargetSlotConstraint_t *)0) {
    motion = (ApaTargetSlotMotion_t){0};
    motion.StartX = fromNode->X;
    motion.StartY = fromNode->Y;
    motion.StartTheta = fromNode->Theta;
    motion.Direction = 1;
    if (toNode->D < 0.0f) {
      motion.Direction = -1;
    }
    motion.Length = fabsf(toNode->D);
    motion.Curvature = tanf(toNode->Delta) / config->Veh.WB;
    apaTargetSlotConstraintCheckMotion(context->SlotConstraint, &motion, &frame);
    context->CheckCount = context->CheckCount + frame.CheckCount;
    collision0 = !frame.IsClear;
  }
  return collision0;
}

static Boolean geoVtPathPlanningHybridCollisionCheck(const ApaHybridNode_t *fromNode,
                                                     const ApaHybridNode_t *toNode,
                                                     void *userData) {
  GeoVtPathPlanningHybridCollisionContext_t *context;
  const ApaPlannerConfig_t *plannerConfig;
  ApaVehicleCollisionCheckResult_t collisionResult;
  Real detectStep;
  Real travelDistance;
  Real collisionTravel;
  Integer sampleCount;
  Boolean collisionDetected;
  Boolean canCheck0;
  Boolean result0;

  context = (GeoVtPathPlanningHybridCollisionContext_t *)userData;
  canCheck0 = (context != (GeoVtPathPlanningHybridCollisionContext_t *)0);
  if (canCheck0) {
    canCheck0 = (context->CollisionContext.PlannerGlobal != (ApaPlannerGlobalState_t *)0);
  }

  result0 = false;
  if (canCheck0) {
    plannerConfig = apaPlannerGlobalGetPlanner(context->CollisionContext.PlannerGlobal);
    detectStep = plannerConfig->Map.CollisionDetectStepLength;
    if (detectStep <= GEO_EPSILON) {
      detectStep = plannerConfig->Map.XyGridResolution;
    }
    travelDistance = fabsf(toNode->D);
    sampleCount = 1;
    if (detectStep > GEO_EPSILON) {
      sampleCount = (Integer)ceilf(travelDistance / detectStep);
      if (sampleCount < 1) {
        sampleCount = 1;
      }
    }
    context->CheckCount = context->CheckCount + sampleCount;

    apaVehicleCollisionCheckResultInit(&collisionResult);
    collisionTravel = 0.0f;
    collisionDetected = apaVehicleCollisionCheckMotionDetailed(fromNode,
                                                               toNode,
                                                               &context->CollisionContext,
                                                               APA_COLLISION_DEMAND_ALL,
                                                               &collisionResult,
                                                               &collisionTravel);
    if (collisionDetected) {
      result0 = true;
    }
    if (!result0 && context->SceneQuery.CheckPose) {
      VehicleCollisionCheckInput_t pose = context->SceneTemplate;
      Real curvature = 0.0f;
      if (travelDistance > GEO_EPSILON)
        curvature = apaCoordWrapToPi(toNode->Theta - fromNode->Theta) / toNode->D;
      const Real corner = sqrtf(pose.VehLF * pose.VehLF + 0.25f * pose.VehW * pose.VehW);
      const Real sceneStep = 0.05f / (1.0f + corner * fabsf(curvature));
      const Integer samples = (Integer)ceilf(travelDistance / sceneStep);
      for (Integer i = 0; i <= samples && !result0; ++i) {
        const Real ratio = samples > 0 ? (Real)i / (Real)samples : 1.0f;
        const Real distance = toNode->D * ratio;
        pose.VehPosTheta = apaCoordWrapToPi(fromNode->Theta + curvature * distance);
        if (fabsf(curvature) > 1.0e-6f) {
          pose.VehPosX = fromNode->X + (sinf(pose.VehPosTheta) - sinf(fromNode->Theta)) / curvature;
          pose.VehPosY = fromNode->Y + (cosf(fromNode->Theta) - cosf(pose.VehPosTheta)) / curvature;
        } else {
          pose.VehPosX = fromNode->X + ratio * (toNode->X - fromNode->X);
          pose.VehPosY = fromNode->Y + ratio * (toNode->Y - fromNode->Y);
        }
        result0 = context->SceneQuery.CheckPose(&pose, context->SceneQuery.UserData);
        ++context->CheckCount;
      }
    }
    {
      Boolean slotCollision0;
      slotCollision0 =
          geoVtPathPlanningHybridSlotCollision(context, plannerConfig, fromNode, toNode);
      result0 = result0 || slotCollision0;
    }
  }
  return result0;
}

static void geoVtPathPlanningHybridBuildSlot(const GeoVtPathPlanningInput_t *input,
                                             ApaSlot_t *slot) {
  apaSlotInit(slot);
  slot->EnterEdge = input->SltEntr;
  slot->TarPos[0] = input->TarX;
  slot->TarPos[1] = input->TarY;
  slot->TarPos[2] = input->TarTheta;
  slot->SlotStatus = APA_SLOT_STATUS_TARGET_SLOT;
  slot->SlotSubStatus = APA_SLOT_SUB_END_NONE;
  slot->SlotPts[0] = input->SltPt1X;
  slot->SlotPts[1] = input->SltPt1Y;
  slot->SlotPts[2] = input->SltPt2X;
  slot->SlotPts[3] = input->SltPt2Y;
  slot->SlotPts[4] = input->SltPt3X;
  slot->SlotPts[5] = input->SltPt3Y;
  slot->SlotPts[6] = input->SltPt4X;
  slot->SlotPts[7] = input->SltPt4Y;
  slot->SlotClass = apaSlotCalcSlotClass(slot->SlotPts, slot->EnterEdge);
  slot->ParkingType = input->PrkType;
  slot->SurroundingType = input->SurdType;
  slot->ParkingDirec = APA_PARKING_DIREC_REAR;
}

static Boolean geoVtPathPlanningHybridPathIsSafe(GeoVtPathPlanningHybridCollisionContext_t *context,
                                                 const ApaHybridAStarMotionPath_t *motion,
                                                 Integer *plnChkCnt) {
  const ApaPath_t *path = &motion->Path;
  Integer index;
  Integer before = context->CheckCount;
  Boolean clear = path->PathN > 1;
  for (index = 0; clear && index + 1 < path->PathN; ++index) {
    ApaHybridNode_t from = {0};
    ApaHybridNode_t to = {0};
    from.X = path->X[index];
    from.Y = path->Y[index];
    from.Theta = path->Tht[index];
    to.X = path->X[index + 1];
    to.Y = path->Y[index + 1];
    to.Theta = path->Tht[index + 1];
    to.D = path->OdoStep[index];
    if (fabsf(to.D) > GEO_EPSILON) {
      const ApaPlannerConfig_t *config =
          apaPlannerGlobalGetPlanner(context->CollisionContext.PlannerGlobal);
      to.Delta = atanf(config->Veh.WB * motion->Curvature[index]);
    }
    clear = !geoVtPathPlanningHybridCollisionCheck(&from, &to, context);
  }
  *plnChkCnt = context->CheckCount - before;
  return clear;
}

static void geoVtPathPlanningHybridCopySegmentsToOutput(Integer pathSegNum,
                                                        const Integer pathGoDir[APA_PATH_SEG_MAX],
                                                        const Integer pathTrnDir[APA_PATH_SEG_MAX],
                                                        const Real pathCrv[APA_PATH_SEG_MAX],
                                                        const Real pathTht[APA_PATH_SEG_MAX],
                                                        const Real pathLen[APA_PATH_SEG_MAX],
                                                        const Real pathStx[APA_PATH_SEG_MAX],
                                                        const Real pathSty[APA_PATH_SEG_MAX],
                                                        const Real pathStTh[APA_PATH_SEG_MAX],
                                                        const Real pathCC[APA_PATH_CC_MAX],
                                                        Integer stpN,
                                                        Integer plnChkCnt,
                                                        GeoVtPathPlanningOutput_t *output) {
  Integer idx;
  Real pathLenSum;

  *output = (GeoVtPathPlanningOutput_t){0};
  output->PlanFlg = 1;
  output->SegNum = pathSegNum;
  output->StpN = stpN;
  output->PlnChkCnt = plnChkCnt;
  output->PlannerCase = GEO_VT_PLANNER_CASE_HYBRID_FALLBACK;

  pathLenSum = 0.0f;
  idx = 0;
  while (idx < pathSegNum) {
    output->SegGoDir[idx] = pathGoDir[idx];
    output->SegTrnDir[idx] = pathTrnDir[idx];
    output->SegCrv[idx] = pathCrv[idx];
    output->SegTht[idx] = pathTht[idx];
    output->SegLen[idx] = pathLen[idx];
    output->SegStX[idx] = pathStx[idx];
    output->SegStY[idx] = pathSty[idx];
    output->SegStTh[idx] = pathStTh[idx];
    output->SegCcX[idx] = pathCC[idx * 2];
    output->SegCcY[idx] = pathCC[idx * 2 + 1];
    pathLenSum = pathLenSum + pathLen[idx];
    idx = idx + 1;
  }
  output->PathLen = pathLenSum;
}

static Boolean
geoVtPathPlanningHybridBuildCandidateFromMotion(const ApaHybridAStarMotionPath_t *motion,
                                                GeoVtPathPlanningOutput_t *output,
                                                ApaPlanningDiagnostics_t *diagnostic) {
  ApaPlannerHybridSegments_t segments;
  if (!apaPlannerHybridBuildSegmentsFromMotion(motion, VT_SEG_OUT_MAX, &segments, diagnostic))
    return false;
  geoVtPathPlanningHybridCopySegmentsToOutput(segments.SegNum,
                                              segments.PathGoDir,
                                              segments.PathTrnDir,
                                              segments.PathCrv,
                                              segments.PathTht,
                                              segments.PathLenArr,
                                              segments.PathStx,
                                              segments.PathSty,
                                              segments.PathStTh,
                                              segments.PathCC,
                                              segments.StpN,
                                              segments.PlnChkCnt,
                                              output);
  return true;
}

static Real geoVtPathPlanningHybridDetectSideSpace(const GeoVtPathPlanningInput_t *input,
                                                   const ApaPlannerVehConfig_t *vehicle,
                                                   const ApaVehicleCollisionObs_t *obs,
                                                   const ApaPlanningSceneQuery_t *scene,
                                                   Integer side) {
  Real step = side > 0 ? 0.05f : 0.03f;
  Real space = 0.0f;
  Real c = cosf(input->TarTheta);
  Real s = sinf(input->TarTheta);

  for (Real offset = 0.05f; offset < 1.0f; offset += step) {
    Real rectangle[8];
    Boolean collision = false;
    apaCoordBuildRect(input->TarX + 1.5f * c - (Real)side * offset * s,
                      input->TarY + 1.5f * s + (Real)side * offset * c,
                      input->TarTheta,
                      vehicle->LF,
                      vehicle->LB,
                      vehicle->W / 2.0f,
                      rectangle);
    for (Integer i = 0; i < obs->BoundaryLineNum && !collision; ++i) {
      collision = apaVehicleCollisionLineHitsRect(&obs->BoundaryLine[i], rectangle);
    }
    for (Integer i = 0; i < obs->BoundarySeparatePointNum && !collision; ++i) {
      collision = apaVehicleCollisionRectContainsPoint(
          rectangle, obs->BoundarySeparatePoint[i][0], obs->BoundarySeparatePoint[i][1]);
    }
    if (!collision && scene && scene->CheckPose) {
      VehicleCollisionCheckInput_t pose = {0};
      pose.CheckType = VCC_CHECK_TYPE_PLANNING;
      pose.CollisionModel = VCC_COLLISION_MODEL_EIGHT_EDGES;
      pose.VehLF = vehicle->LF;
      pose.VehLB = vehicle->LB;
      pose.VehW = vehicle->W;
      pose.VehWB = vehicle->WB;
      pose.VehLL = vehicle->LF + vehicle->LB;
      pose.VehPosX = input->TarX + 1.5f * c - (Real)side * offset * s;
      pose.VehPosY = input->TarY + 1.5f * s + (Real)side * offset * c;
      pose.VehPosTheta = input->TarTheta;
      collision = scene->CheckPose(&pose, scene->UserData);
    }
    if (collision) {
      break;
    }
    space = offset;
  }
  return space;
}

static void geoVtPathPlanningHybridSearchEndPose(const GeoVtPathPlanningInput_t *input,
                                                 ApaSlot_t *slot,
                                                 const ApaPlannerConfig_t *config,
                                                 const ApaVehicleCollisionObs_t *obs,
                                                 const ApaPlanningSceneQuery_t *scene,
                                                 Real endPose[3]) {
  Real enter[3];
  Real transition[3];
  Real distance = 1.0f;
  apaSlotGetParkingPos(
      slot, config->Conf.EnterPosLen, config->Conf.TransitionPosLen, enter, transition);
  if (input->PrkType == APA_PARKING_TYPE_LEFT || input->PrkType == APA_PARKING_TYPE_RIGHT) {
    if (input->SurdType == APA_SURROUND_OBS_EXIST_BOTH) {
      Real left = geoVtPathPlanningHybridDetectSideSpace(input, &config->Veh, obs, scene, 1);
      Real right = geoVtPathPlanningHybridDetectSideSpace(input, &config->Veh, obs, scene, -1);
      distance = apaMathMin(left, right);
    } else if (input->PrkType == APA_PARKING_TYPE_LEFT &&
               input->SurdType == APA_SURROUND_OBS_EXIST_LEFT_FRONT_ONLY) {
      distance = geoVtPathPlanningHybridDetectSideSpace(input, &config->Veh, obs, scene, 1);
    } else if (input->PrkType == APA_PARKING_TYPE_RIGHT &&
               input->SurdType == APA_SURROUND_OBS_EXIST_RIGHT_REAR_ONLY) {
      distance = geoVtPathPlanningHybridDetectSideSpace(input, &config->Veh, obs, scene, -1);
    }
  }
  if (distance > 0.5f) {
    Real length = 2.0f + (config->Conf.EnterPosLen - 2.0f) * (1.0f - distance);
    enter[0] = input->TarX + length * cosf(input->TarTheta);
    enter[1] = input->TarY + length * sinf(input->TarTheta);
  }
  for (Integer i = 0; i < 3; ++i) {
    if (input->PrkType == APA_PARKING_TYPE_MEANINGLESS) {
      endPose[i] = (enter[i] + transition[i]) / 2.0f;
    } else if (input->SurdType == APA_SURROUND_OBS_EXIST_NONE ||
               (input->PrkType == APA_PARKING_TYPE_LEFT &&
                input->SurdType == APA_SURROUND_OBS_EXIST_RIGHT_REAR_ONLY) ||
               (input->PrkType == APA_PARKING_TYPE_RIGHT &&
                input->SurdType == APA_SURROUND_OBS_EXIST_LEFT_FRONT_ONLY)) {
      endPose[i] = transition[i];
    } else {
      endPose[i] = enter[i];
    }
  }

  const Real c = cosf(input->TarTheta), s = sinf(input->TarTheta);
  const Real along = c * (endPose[0] - input->TarX) + s * (endPose[1] - input->TarY);
  endPose[0] = input->TarX + along * c;
  endPose[1] = input->TarY + along * s;
  endPose[2] = input->TarTheta;
}

static Boolean geoVtPathPlanningHybridCompleteMotion(const GeoVtPathPlanningInput_t *input,
                                                     const ApaHybridAStarMotionPath_t *source,
                                                     ApaHybridAStarMotionPath_t *motion) {
  *motion = *source;
  ApaPath_t *path = &motion->Path;
  const Integer originalN = path->PathN;
  if (originalN < 2)
    return false;
  apaPathReverse(path);
  for (Integer i = 0; i + 1 < originalN; ++i)
    motion->Curvature[i] = source->Curvature[originalN - 2 - i];
  Real dx = path->X[0] - input->VehX, dy = path->Y[0] - input->VehY;
  Real distance = apaMathNorm2D(dx, dy);
  Real yawError = fabsf(apaCoordWrapToPi(path->Tht[0] - input->VehTheta));
  if (distance > GEO_VT_HYBRID_FIT_POS_EPS || yawError > GEO_VT_HYBRID_FIT_YAW_EPS) {
    const Real c = cosf(input->VehTheta), s = sinf(input->VehTheta);
    if (fabsf(c * dy - s * dx) > 1.0e-4f || yawError > 1.0e-4f ||
        path->PathN >= APA_PATH_MAX_POINTS)
      return false;
    for (Integer i = path->PathN - 2; i >= 0; --i)
      motion->Curvature[i + 1] = motion->Curvature[i];
    apaPathPrependPos(path, input->VehX, input->VehY, input->VehTheta);
    path->OdoStep[0] = c * dx + s * dy;
    motion->Curvature[0] = 0.0f;
  }
  const Integer n = path->PathN;
  dx = input->TarX - path->X[n - 1];
  dy = input->TarY - path->Y[n - 1];
  distance = apaMathNorm2D(dx, dy);
  yawError = fabsf(apaCoordWrapToPi(path->Tht[n - 1] - input->TarTheta));
  if (distance > GEO_VT_HYBRID_FIT_POS_EPS || yawError > GEO_VT_HYBRID_FIT_YAW_EPS) {
    const Real c = cosf(input->TarTheta), s = sinf(input->TarTheta);
    if (fabsf(c * dy - s * dx) > 1.0e-4f || yawError > 1.0e-4f || n >= APA_PATH_MAX_POINTS)
      return false;
    apaPathAppendPos(path, input->TarX, input->TarY, input->TarTheta);
    path->OdoStep[n - 1] = c * dx + s * dy;
    motion->Curvature[n - 1] = 0.0f;
  }
  return true;
}

typedef struct {
  const GeoVtPathPlanningInput_t *Input;
  const GeoVtPathPlanningRunParam_t *Param;
  GeoVtPathPlanningHybridCollisionContext_t *Collision;
  GeoVtPathPlanningOutput_t Best;
  GeoVtPathPlanningOutput_t Last;
  Integer Accepted;
  Integer Rejected;
} GeoVtPathPlanningHybridCandidateContext_t;

static Boolean geoVtPathPlanningHybridAcceptMotionPath(const ApaHybridAStarMotionPath_t *source,
                                                       ApaPath_t *qualityPath,
                                                       void *userData) {
  GeoVtPathPlanningHybridCandidateContext_t *context = userData;
  ApaHybridAStarMotionPath_t motion;
  GeoVtPathPlanningOutput_t candidate = {0};
  Integer rejectedSegment = 0, checks = 0;
  const char *reason = "completion_alignment";
  Boolean accepted = geoVtPathPlanningHybridCompleteMotion(context->Input, source, &motion);
  if (accepted) {
    reason = "primitive_geometry_or_capacity";
    accepted = geoVtPathPlanningHybridBuildCandidateFromMotion(
        &motion, &candidate, context->Param->Base.Diagnostics);
  }
  if (accepted) {
    reason = "segment_continuity";
    accepted = geoVtPathPlanningGeometryIsContinuous(context->Input, &candidate, &rejectedSegment);
  }
  if (accepted) {
    reason = "native_motion_collision";
    accepted = geoVtPathPlanningHybridPathIsSafe(context->Collision, &motion, &checks);
    candidate.PlnChkCnt += checks;
  }
  if (accepted) {
    reason = "candidate_safety_or_policy";
    checks = 0;
    accepted =
        geoVtPathPlanningIsCandidateSafe(context->Input, context->Param, &candidate, &checks);
    candidate.PlnChkCnt += checks;
  }
  if (accepted) {
    ++context->Accepted;
    context->Last = candidate;
    *qualityPath = motion.Path;
    qualityPath->CollisionFlag = APA_COLLISION_RISK_NOT_EXIST;
  } else {
    ++context->Rejected;
    apaPlanningRecordFailure(
        context->Param->Base.Diagnostics, APA_FAULT_PATH_NOT_MATCH_TARPOS_IN_LOCAL, reason, 3);
    if (context->Rejected <= 8)
      fprintf(stderr,
              "[APA_VT_CANDIDATE] accepted=0 reason=%s segment=%ld "
              "rejection=%ld continue_search=1\n",
              reason,
              (long)rejectedSegment,
              (long)context->Rejected);
  }
  return accepted;
}

static void geoVtPathPlanningHybridSelectMotion(void *userData) {
  GeoVtPathPlanningHybridCandidateContext_t *context = userData;
  context->Best = context->Last;
}

static Boolean geoVtPathPlanningTryHybridFallback(const GeoVtPathPlanningInput_t *input,
                                                  const GeoVtPathPlanningRunParam_t *param,
                                                  GeoVtPathPlanningOutput_t *output) {
  if (!apaPlannerHybridResolveFusion(param->Base.NeedHybridAlgFusion))
    return false;
  ApaPlannerGlobalState_t plannerGlobal = APA_PLANNER_GLOBAL_STATE_INIT;
  GeoVtPathPlanningHybridCollisionContext_t collisionContext = {0};
  GeoVtPathPlanningHybridCandidateContext_t candidates = {0};
  ApaHybridAStar_t planner = {0};
  ApaSlot_t slot = {0};
  Real startPose[3] = {input->VehX, input->VehY, input->VehTheta};
  Real goalPose[3];
  geoVtPathPlanningHybridInitPlannerGlobal(input, param, &plannerGlobal);
  geoVtPathPlanningHybridBuildSlot(input, &slot);
  geoVtPathPlanningHybridInitCollisionContext(
      input, &slot, param, &plannerGlobal, &collisionContext);
  geoVtPathPlanningHybridSearchEndPose(input,
                                       &slot,
                                       &plannerGlobal.Planner,
                                       &collisionContext.CollisionContext.Obs,
                                       &param->Base.SceneQuery,
                                       goalPose);
  VehicleCollisionCheckInput_t precheck;
  VehicleCollisionCheckOutput_t checked = {0};
  geoVtPathPlanningBuildTargetPrecheckCollisionInput(
      input, param, goalPose[0], goalPose[1], goalPose[2], &precheck);
  apaPlanningSceneCollisionCheck(&precheck, &param->Base.SceneQuery, &checked);
  if (checked.IsCollision) {
    apaPlanningRecordFailure(param->Base.Diagnostics,
                             APA_FAULT_COLLISION_AT_TARGET_SLOT,
                             "hybrid_alignment_precheck",
                             5);
    return false;
  }

  apaHybridAStarInit(&planner);
  apaHybridAStarSetPlannerGlobal(&planner, &plannerGlobal);
  apaHybridAStarLoadSlot(&planner, &slot);
  planner.ReversePlanFlag = true;
  planner.GridSpec.OriginX =
      0.5f * (input->VehX + input->TarX) -
      0.5f * (Real)(planner.GridSpec.XCount - 1) * planner.GridSpec.XResolution;
  planner.GridSpec.OriginY =
      0.5f * (input->VehY + input->TarY) -
      0.5f * (Real)(planner.GridSpec.YCount - 1) * planner.GridSpec.YResolution;
  planner.Diagnostics = param->Base.Diagnostics;
  if (!apaHybridAStarLoadStartGoalPoses(&planner, goalPose, startPose)) {
    apaPlanningRecordFailure(
        planner.Diagnostics, apaPlanningSearchError(planner.Fault), "hybrid_grid", 2);
    return false;
  }
  apaPlannerHybridConfigureQuality(&planner, &collisionContext.CollisionContext.Obs);
  apaHybridAStarSetCollisionCheck(
      &planner, geoVtPathPlanningHybridCollisionCheck, &collisionContext);
  candidates.Input = input;
  candidates.Param = param;
  candidates.Collision = &collisionContext;
  planner.CandidateCheck = geoVtPathPlanningHybridAcceptMotionPath;
  planner.CandidateSelected = geoVtPathPlanningHybridSelectMotion;
  planner.CandidateUserData = &candidates;
  const ApaHybridAStarState_e runState = apaHybridAStarRunToTerminal(&planner);
  fprintf(
      stderr,
      "[APA_VT_SEARCH] stage=HYBRID state=%d fault=%d iterations=%ld "
      "origin_x=%.6f origin_y=%.6f start_x=%.6f start_y=%.6f goal_x=%.6f goal_y=%.6f "
      "align_x=%.6f align_y=%.6f reverse_search=1 accepted=%ld rejected=%ld goal_rejections=%ld "
      "nodes=%ld truncated=%ld expected_segments=%ld quality_min_nodes=%ld max_nodes=%ld\n",
      (int)runState,
      (int)planner.Fault,
      (long)planner.ExpandIterations,
      (double)planner.GridSpec.OriginX,
      (double)planner.GridSpec.OriginY,
      (double)startPose[0],
      (double)startPose[1],
      (double)input->TarX,
      (double)input->TarY,
      (double)goalPose[0],
      (double)goalPose[1],
      (long)candidates.Accepted,
      (long)candidates.Rejected,
      (long)planner.RejectedGoalCandidates,
      (long)planner.NodePool.ValidNodeCnt,
      (long)planner.TruncatedPrimitives,
      (long)planner.ExpectedSegmentCount,
      (long)apaHybridAStarQualityMinimumNodes(&planner),
      (long)plannerGlobal.Planner.Conf.PlanningCalPowerMaxPerSlot);
  if (runState != APA_HYBRID_STATE_GOAL_REACHED || candidates.Accepted == 0)
    return false;
  candidates.Best.PlnChkCnt += collisionContext.CheckCount;
  *output = candidates.Best;
  return true;
}

static void geoVtPathPlanningZeroPathArrays(Integer pathGoDir[APA_PATH_SEG_MAX],
                                            Integer pathTrnDir[APA_PATH_SEG_MAX],
                                            Real pathCrv[APA_PATH_SEG_MAX],
                                            Real pathTht[APA_PATH_SEG_MAX],
                                            Real pathLen[APA_PATH_SEG_MAX],
                                            Real pathStx[APA_PATH_SEG_MAX],
                                            Real pathSty[APA_PATH_SEG_MAX],
                                            Real pathStTh[APA_PATH_SEG_MAX],
                                            Real pathCC[APA_PATH_CC_MAX]) {
  Integer idx;

  idx = 0;
  while (idx < APA_PATH_SEG_MAX) {
    pathGoDir[idx] = 0;
    pathTrnDir[idx] = 0;
    pathCrv[idx] = 0.0f;
    pathTht[idx] = 0.0f;
    pathLen[idx] = 0.0f;
    pathStx[idx] = 0.0f;
    pathSty[idx] = 0.0f;
    pathStTh[idx] = 0.0f;
    idx = idx + 1;
  }

  idx = 0;
  while (idx < APA_PATH_CC_MAX) {
    pathCC[idx] = 0.0f;
    idx = idx + 1;
  }
}

static void geoVtPathPlanningExportCandidate(const GeoVtPathPlanningOutput_t *candidate,
                                             Integer *pathSegNum,
                                             Integer pathGoDir[APA_PATH_SEG_MAX],
                                             Integer pathTrnDir[APA_PATH_SEG_MAX],
                                             Real pathCrv[APA_PATH_SEG_MAX],
                                             Real pathTht[APA_PATH_SEG_MAX],
                                             Real pathLen[APA_PATH_SEG_MAX],
                                             Real pathStx[APA_PATH_SEG_MAX],
                                             Real pathSty[APA_PATH_SEG_MAX],
                                             Real pathStTh[APA_PATH_SEG_MAX],
                                             Real pathCC[APA_PATH_CC_MAX]) {
  geoVtPathPlanningZeroPathArrays(
      pathGoDir, pathTrnDir, pathCrv, pathTht, pathLen, pathStx, pathSty, pathStTh, pathCC);
  *pathSegNum = candidate->SegNum;

  {
    Integer si;
    si = 0;
    while (si < *pathSegNum && si < VT_SEG_OUT_MAX) {
      pathGoDir[si] = candidate->SegGoDir[si];
      pathTrnDir[si] = candidate->SegTrnDir[si];
      pathCrv[si] = candidate->SegCrv[si];
      pathTht[si] = candidate->SegTht[si];
      pathLen[si] = candidate->SegLen[si];
      pathStx[si] = candidate->SegStX[si];
      pathSty[si] = candidate->SegStY[si];
      pathStTh[si] = candidate->SegStTh[si];
      pathCC[si * 2] = candidate->SegCcX[si];
      pathCC[si * 2 + 1] = candidate->SegCcY[si];
      si = si + 1;
    }
  }
}

static inline void geoVtPathPlanningLegacyFallbackCalcArcs(Integer d,
                                                           Real posRx,
                                                           Real posRy,
                                                           Real posRtheta,
                                                           Real rMin,
                                                           Boolean *ok,
                                                           Integer *td1,
                                                           Integer *td2,
                                                           Real *c1x,
                                                           Real *c1y,
                                                           Real *c2x,
                                                           Real *c2y,
                                                           Real *s2x,
                                                           Real *s2y,
                                                           Real *s2th,
                                                           Real *t1,
                                                           Real *t2) {
  Real ct2Dy, sqrtArg, cosArg;
  if (d < 0) {
    *td1 = -1;
    *td2 = 1;
    *c1x = posRx + rMin * cosf(posRtheta - APA_HALF_PI);
    *c1y = posRy + rMin * sinf(posRtheta - APA_HALF_PI);
    *c2y = rMin;
    ct2Dy = *c2y - *c1y;
    if ((rMin + rMin) < fabsf(ct2Dy)) {
      *ok = false;
    } else {
      sqrtArg = (rMin + rMin) * (rMin + rMin) - ct2Dy * ct2Dy;
      if (sqrtArg < 0.0f) {
        sqrtArg = 0.0f;
      }
      *c2x = *c1x + sqrtf(sqrtArg);
      cosArg = ct2Dy / (rMin + rMin);
      if (cosArg > 1.0f) {
        cosArg = 1.0f;
      }
      if (cosArg < -1.0f) {
        cosArg = -1.0f;
      }
      *t2 = -acosf(cosArg);
      *t1 = posRtheta - *t2;
      *s2x = *c2x + rMin * cosf(*t2 - APA_HALF_PI);
      *s2y = *c2y + rMin * sinf(*t2 - APA_HALF_PI);
      *s2th = *t2;
    }
  } else {
    *td1 = 1;
    *td2 = -1;
    *c1x = posRx + rMin * cosf(posRtheta + APA_HALF_PI);
    *c1y = posRy + rMin * sinf(posRtheta + APA_HALF_PI);
    *c2y = -rMin;
    ct2Dy = *c2y - *c1y;
    if ((rMin + rMin) < fabsf(ct2Dy)) {
      *ok = false;
    } else {
      sqrtArg = (rMin + rMin) * (rMin + rMin) - ct2Dy * ct2Dy;
      if (sqrtArg < 0.0f) {
        sqrtArg = 0.0f;
      }
      *c2x = *c1x + sqrtf(sqrtArg);
      cosArg = (*c1y - *c2y) / (rMin + rMin);
      if (cosArg > 1.0f) {
        cosArg = 1.0f;
      }
      if (cosArg < -1.0f) {
        cosArg = -1.0f;
      }
      *t2 = acosf(cosArg);
      *t1 = posRtheta - *t2;
      *s2x = *c2x + rMin * cosf(*t2 + APA_HALF_PI);
      *s2y = *c2y + rMin * sinf(*t2 + APA_HALF_PI);
      *s2th = *t2;
    }
  }
}

static Boolean geoVtPathPlanningLegacyFallback(const GeoVtPathPlanningInput_t *input,
                                               Real anchorX,
                                               Real anchorY,
                                               GeoVtPathPlanningOutput_t *output) {
  Real vehX;
  Real vehY;
  Real vehTheta;
  Real tarX;
  Real tarY;
  Real tarTheta;
  Real rMin;
  Real entrX;
  Real entrY;
  Boolean planOk;
  Real posRx;
  Real posRy;
  Real posRtheta;
  Integer trnD1;
  Integer trnD2;
  Real ct1rx;
  Real ct1ry;
  Real ct2rx;
  Real ct2ry;
  Real st2rx;
  Real st2ry;
  Real st2rTheta;
  Real st3rx;
  Real st3ry;
  Real st3rTheta;
  Real tht1;
  Real tht2;

  vehX = input->VehX;
  vehY = input->VehY;
  vehTheta = input->VehTheta;
  tarX = input->TarX;
  tarY = input->TarY;
  tarTheta = input->TarTheta;
  rMin = input->Rmin;
  entrX = anchorX;
  entrY = anchorY;
  planOk = false;

  if (rMin > GEO_EPSILON) {
    geoAxisTransToTarget(vehX, vehY, vehTheta, entrX, entrY, tarTheta, &posRx, &posRy, &posRtheta);

    trnD1 = 0;
    trnD2 = 0;
    ct1rx = 0.0f;
    ct1ry = 0.0f;
    ct2rx = 0.0f;
    ct2ry = 0.0f;
    st2rx = 0.0f;
    st2ry = 0.0f;
    st2rTheta = 0.0f;
    st3rx = 0.0f;
    st3ry = 0.0f;
    st3rTheta = 0.0f;
    tht1 = 0.0f;
    tht2 = 0.0f;
    {
      Real bestArcLen;
      Integer dirs[2];
      Integer di;

      if (posRy > 0.0f) {
        dirs[0] = -1;
        dirs[1] = 1;
      } else {
        dirs[0] = 1;
        dirs[1] = -1;
      }

      bestArcLen = 1.0e30f;

      di = 0;
      while (di < 2) {
        Integer d;
        Integer td1;
        Integer td2;
        Real c1x;
        Real c1y;
        Real c2x;
        Real c2y;
        Real s2x;
        Real s2y;
        Real s2th;
        Real t1;
        Real t2;

        Real arcSum;
        Boolean ok;

        d = dirs[di];
        ok = true;

        geoVtPathPlanningLegacyFallbackCalcArcs(d,
                                                posRx,
                                                posRy,
                                                posRtheta,
                                                rMin,
                                                &ok,
                                                &td1,
                                                &td2,
                                                &c1x,
                                                &c1y,
                                                &c2x,
                                                &c2y,
                                                &s2x,
                                                &s2y,
                                                &s2th,
                                                &t1,
                                                &t2);

        if (ok) {
          arcSum = fabsf(t1 * rMin) + fabsf(t2 * rMin);
          if (arcSum < bestArcLen) {
            bestArcLen = arcSum;
            trnD1 = td1;
            trnD2 = td2;
            ct1rx = c1x;
            ct1ry = c1y;
            ct2rx = c2x;
            ct2ry = c2y;
            st2rx = s2x;
            st2ry = s2y;
            st2rTheta = s2th;
            st3rx = c2x;
            st3ry = 0.0f;
            st3rTheta = 0.0f;
            tht1 = t1;
            tht2 = t2;
            planOk = true;
          }
        }

        di = di + 1;
      }
    }

    if (planOk) {
      Real g1x;
      Real g1y;
      Real g2x;
      Real g2y;
      Real gSt2x;
      Real gSt2y;
      Real gSt2theta;
      Real gSt3x;
      Real gSt3y;
      Real gSt3theta;
      Real arc1Len;
      Real arc2Len;
      Real straightLen;
      Real cosE;
      Real sinE;

      cosE = cosf(tarTheta);
      sinE = sinf(tarTheta);

      g1x = entrX + cosE * ct1rx - sinE * ct1ry;
      g1y = entrY + sinE * ct1rx + cosE * ct1ry;
      g2x = entrX + cosE * ct2rx - sinE * ct2ry;
      g2y = entrY + sinE * ct2rx + cosE * ct2ry;
      gSt2x = entrX + cosE * st2rx - sinE * st2ry;
      gSt2y = entrY + sinE * st2rx + cosE * st2ry;
      gSt2theta = tarTheta + st2rTheta;
      gSt3x = entrX + cosE * st3rx - sinE * st3ry;
      gSt3y = entrY + sinE * st3rx + cosE * st3ry;
      gSt3theta = tarTheta + st3rTheta;

      arc1Len = fabsf(tht1 * rMin);
      arc2Len = fabsf(tht2 * rMin);
      straightLen = sqrtf((gSt3x - tarX) * (gSt3x - tarX) + (gSt3y - tarY) * (gSt3y - tarY));

      output->PlanFlg = 1;
      output->SegNum = 3;
      output->PathLen = arc1Len + arc2Len + straightLen;

      {
        Integer goDir1;
        Integer goDir2;
        Integer goDir3;
        Real straightProjection0;

        goDir1 = (-tht1 * (Real)trnD1 < 0.0f) ? -1 : 1;
        goDir2 = (-tht2 * (Real)trnD2 < 0.0f) ? -1 : 1;
        straightProjection0 = cosf(gSt3theta) * (tarX - gSt3x) + sinf(gSt3theta) * (tarY - gSt3y);
        goDir3 = (straightProjection0 < 0.0f) ? -1 : 1;

        output->SegGoDir[0] = goDir1;
        output->SegTrnDir[0] = trnD1;
        output->SegCrv[0] = ((Real)trnD1) / rMin;
        output->SegTht[0] = (Real)trnD1 * fabsf(tht1);
        output->SegLen[0] = arc1Len;
        output->SegStX[0] = vehX;
        output->SegStY[0] = vehY;
        output->SegStTh[0] = vehTheta;
        output->SegCcX[0] = g1x;
        output->SegCcY[0] = g1y;

        output->SegGoDir[1] = goDir2;
        output->SegTrnDir[1] = trnD2;
        output->SegCrv[1] = ((Real)trnD2) / rMin;
        output->SegTht[1] = (Real)trnD2 * fabsf(tht2);
        output->SegLen[1] = arc2Len;
        output->SegStX[1] = gSt2x;
        output->SegStY[1] = gSt2y;
        output->SegStTh[1] = gSt2theta;
        output->SegCcX[1] = g2x;
        output->SegCcY[1] = g2y;

        output->SegGoDir[2] = goDir3;
        output->SegTrnDir[2] = 0;
        output->SegCrv[2] = 0.0f;
        output->SegTht[2] = 0.0f;
        output->SegLen[2] = straightLen;
        output->SegStX[2] = gSt3x;
        output->SegStY[2] = gSt3y;
        output->SegStTh[2] = gSt3theta;
        output->SegCcX[2] = 0.0f;
        output->SegCcY[2] = 0.0f;
        output->StpN = geoVtPathPlanningMainLoopCountStpN(output);
      }
    }
  }

  return planOk;
}

static Boolean geoVtPathPlanningCircle2lineFallback(const GeoVtPathPlanningInput_t *input,
                                                    const GeoVtPathPlanningRunParam_t *param,
                                                    GeoVtPathPlanningOutput_t *output) {

  Real rMin;
  Integer dirTry;
  Boolean success0;

  rMin = input->Rmin;
  success0 = false;

  dirTry = 1;
  if (rMin <= GEO_EPSILON) {
    dirTry = -3;
  }
  for (; dirTry >= -1; dirTry = dirTry - 2) {
    GeoCircle1pt1lineInput_t c1In;
    GeoCircle1pt1lineOutput_t c1Out;
    Real radius;
    Boolean radiusValid;

    c1In = (GeoCircle1pt1lineInput_t){0};
    c1Out = (GeoCircle1pt1lineOutput_t){0};
    c1In.VstX = input->VehX;
    c1In.VstY = input->VehY;
    c1In.VstTheta = input->VehTheta;
    c1In.VtgtX = input->TarX;
    c1In.VtgtY = input->TarY;
    c1In.VtgtTheta = input->TarTheta;
    c1In.Dir = dirTry;
    geoCircle1pt1line(&c1In, &c1Out);

    radius = fabsf(c1Out.Radius);
    radiusValid = (radius >= rMin) && (radius <= 1.0e6f);

    if (radiusValid) {

      {
        Integer ptypTry;
        Integer ptypList[2];
        Integer pi;

        ptypList[0] = 12;
        ptypList[1] = 42;

        for (pi = 0; pi < 2; pi = pi + 1) {
          GeoCircle2lineRInput_t c2In;
          GeoCircle2lineROutput_t c2Out;
          Real useR;
          Real lineLen;
          Real lineDx;
          Real lineDy;
          Real lineRelX;
          Integer lineGoDir;
          Boolean geomValid;

          ptypTry = ptypList[pi];
          useR = rMin;
          c2In = (GeoCircle2lineRInput_t){0};
          c2Out = (GeoCircle2lineROutput_t){0};
          c2In.VstX = input->VehX;
          c2In.VstY = input->VehY;
          c2In.VstTheta = input->VehTheta;
          c2In.VtgtX = input->TarX;
          c2In.VtgtY = input->TarY;
          c2In.VtgtTheta = input->TarTheta;
          c2In.Ptyp = ptypTry;
          c2In.Dir = dirTry;
          c2In.Radius = useR;
          geoCircle2lineR(&c2In, &c2Out);

          lineDx = c2Out.StPtX - input->VehX;
          lineDy = c2Out.StPtY - input->VehY;
          lineLen = sqrtf(lineDx * lineDx + lineDy * lineDy);

          lineRelX = cosf(input->VehTheta) * lineDx + sinf(input->VehTheta) * lineDy;
          if (lineRelX > GEO_EPSILON) {
            lineGoDir = 1;
          } else if (lineRelX < -GEO_EPSILON) {
            lineGoDir = -1;
          } else {
            lineGoDir = 1;
          }

          geomValid = (lineLen >= GEO_EPSILON) && (c2Out.ArcLen >= GEO_EPSILON);

          if (geomValid) {

            Boolean lineCollisionReject;
            Boolean arcCollisionReject;

            lineCollisionReject = false;
            {
              GeoLineIntrvCheckInput_t lineChkIn;
              GeoLineIntrvCheckOutput_t lineChkOut;

              geoVtPathPlanningFillLineCheck(input,
                                             param,
                                             input->VehX,
                                             input->VehY,
                                             input->VehTheta,
                                             c2Out.StPtX,
                                             c2Out.StPtY,
                                             c2Out.StPtTheta,
                                             0.2f,
                                             VCC_SLOT_CLASS_VERTICAL,
                                             &lineChkIn);
              lineChkOut = (GeoLineIntrvCheckOutput_t){0};
              geoVtPathPlanningCheckLineWithPolicy(&lineChkIn, &lineChkOut, param);

              if (lineChkOut.IntrvFlg != 0) {
                lineCollisionReject = true;
              }
            }

            arcCollisionReject = false;
            if (!lineCollisionReject) {
              {
                GeoCircleIntrvCheckInput_t arcChkIn;
                GeoCircleIntrvCheckOutput_t arcChkOut;

                geoVtPathPlanningFillCircleCheck(input,
                                                 param,
                                                 c2Out.CptX,
                                                 c2Out.CptY,
                                                 useR,
                                                 c2Out.StPtX,
                                                 c2Out.StPtY,
                                                 c2Out.StPtTheta,
                                                 c2Out.EdPtX,
                                                 c2Out.EdPtY,
                                                 c2Out.EdPtTheta,
                                                 c2Out.GoDir,
                                                 c2Out.TrnDir,
                                                 GEO_VT_FRONT_ARC_CHECK_STEP,
                                                 &arcChkIn);
                arcChkOut = (GeoCircleIntrvCheckOutput_t){0};
                geoVtPathPlanningCheckCircleWithPolicy(&arcChkIn, &arcChkOut, param);

                if (arcChkOut.IntrvFlg != 0) {
                  arcCollisionReject = true;
                }
              }
            }

            if (!lineCollisionReject && !arcCollisionReject) {
              *output = (GeoVtPathPlanningOutput_t){0};
              output->PlanFlg = 1;
              output->SegNum = 2;
              output->PathLen = lineLen + c2Out.ArcLen;

              output->SegGoDir[0] = lineGoDir;
              output->SegTrnDir[0] = 0;
              output->SegCrv[0] = 0.0f;
              output->SegTht[0] = 0.0f;
              output->SegLen[0] = lineLen;
              output->SegStX[0] = input->VehX;
              output->SegStY[0] = input->VehY;
              output->SegStTh[0] = input->VehTheta;
              output->SegCcX[0] = 0.0f;
              output->SegCcY[0] = 0.0f;

              output->SegGoDir[1] = c2Out.GoDir;
              output->SegTrnDir[1] = c2Out.TrnDir;
              output->SegCrv[1] = ((Real)c2Out.TrnDir) / useR;
              output->SegTht[1] =
                  geoVtPathPlanningArcTht(c2Out.StPtTheta, c2Out.EdPtTheta, c2Out.GoDir);
              output->SegLen[1] = c2Out.ArcLen;
              output->SegStX[1] = c2Out.StPtX;
              output->SegStY[1] = c2Out.StPtY;
              output->SegStTh[1] = c2Out.StPtTheta;
              output->SegCcX[1] = c2Out.CptX;
              output->SegCcY[1] = c2Out.CptY;

              success0 = true;
              pi = 2;
              dirTry = -3;
            }
          }
        }
      }
    }
  }

  return success0;
}

static Boolean geoVtPathPlanningTryBestSafeLegacyFallback(const GeoVtPathPlanningInput_t *input,
                                                          const GeoVtPathPlanningRunParam_t *param,
                                                          GeoVtPathPlanningOutput_t *output) {
  const Real blendList[6] = {0.0f, 0.08f, 0.15f, 0.22f, 0.30f, 0.38f};
  Integer idx;
  Boolean foundSafe;
  GeoVtPathPlanningOutput_t bestSafe;

  foundSafe = false;
  bestSafe = (GeoVtPathPlanningOutput_t){0};
  idx = 0;
  while (idx < 6) {
    Real anchorX;
    Real anchorY;
    GeoVtPathPlanningOutput_t candidate;
    Integer chkCnt;
    Boolean buildOk;
    Boolean safeOk;

    anchorX = input->EntrPtX;
    anchorY = input->EntrPtY;
    if (blendList[idx] > GEO_EPSILON) {
      geoVtPathPlanningBuildAnchor(input, blendList[idx], &anchorX, &anchorY);
    }

    candidate = (GeoVtPathPlanningOutput_t){0};
    chkCnt = 0;
    buildOk = geoVtPathPlanningLegacyFallback(input, anchorX, anchorY, &candidate);
    if (buildOk && geoVtPathPlanningHasValidCandidate(&candidate)) {
      safeOk = geoVtPathPlanningIsCandidateSafe(input, param, &candidate, &chkCnt);
      candidate.PlnChkCnt = candidate.PlnChkCnt + chkCnt;
      if (safeOk) {
        if (geoVtPathPlanningShouldReplaceCandidate(&bestSafe, &candidate, param)) {
          bestSafe = candidate;
        }
        foundSafe = true;
      }
    }

    idx = idx + 1;
  }

  if (foundSafe) {
    *output = bestSafe;
  }

  return foundSafe;
}

static Boolean geoVtPathPlanningTrySafeGenericFallback(const GeoVtPathPlanningInput_t *input,
                                                       const GeoVtPathPlanningRunParam_t *param,
                                                       Integer startPosTyp,
                                                       GeoVtPathPlanningOutput_t *output) {
  GeoVtPathPlanningOutput_t candidate;
  Boolean fallbackOk;

  candidate = (GeoVtPathPlanningOutput_t){0};
  fallbackOk = geoVtPathPlanningTryBestSafeLegacyFallback(input, param, &candidate);
  if (fallbackOk) {
    *output = candidate;
  }

  (void)startPosTyp;
  return fallbackOk;
}

static Boolean geoVtPathPlanningTrySafeCircle2lineFallback(const GeoVtPathPlanningInput_t *input,
                                                           const GeoVtPathPlanningRunParam_t *param,
                                                           GeoVtPathPlanningOutput_t *output) {
  GeoVtPathPlanningOutput_t candidate;
  Integer chkCnt;
  Boolean buildOk;
  Boolean safeOk;

  candidate = (GeoVtPathPlanningOutput_t){0};
  chkCnt = 0;
  buildOk = geoVtPathPlanningCircle2lineFallback(input, param, &candidate);
  safeOk = false;
  if (buildOk && geoVtPathPlanningHasValidCandidate(&candidate)) {
    safeOk = geoVtPathPlanningIsCandidateSafe(input, param, &candidate, &chkCnt);
    candidate.PlnChkCnt = candidate.PlnChkCnt + chkCnt;
    if (safeOk) {
      *output = candidate;
    }
  }

  return safeOk;
}

static Boolean geoVtPathPlanningTryBestGeometricFallback(const GeoVtPathPlanningInput_t *input,
                                                         const GeoVtPathPlanningRunParam_t *param,
                                                         Integer startPosTyp,
                                                         GeoVtPathPlanningOutput_t *output) {
  GeoVtPathPlanningOutput_t genericCandidate;
  GeoVtPathPlanningOutput_t circleCandidate;
  GeoVtPathPlanningOutput_t bestCandidate;
  Boolean genericOk;
  Boolean circleOk;

  genericCandidate = (GeoVtPathPlanningOutput_t){0};
  circleCandidate = (GeoVtPathPlanningOutput_t){0};
  bestCandidate = (GeoVtPathPlanningOutput_t){0};
  genericOk = geoVtPathPlanningTrySafeGenericFallback(input, param, startPosTyp, &genericCandidate);
  circleOk = geoVtPathPlanningTrySafeCircle2lineFallback(input, param, &circleCandidate);

  if (genericOk) {
    bestCandidate = genericCandidate;
  }
  if (geoVtPathPlanningShouldReplaceCandidate(&bestCandidate, &circleCandidate, param)) {
    bestCandidate = circleCandidate;
  }

  if (geoVtPathPlanningHasValidCandidate(&bestCandidate)) {
    *output = bestCandidate;
  }

  return genericOk || circleOk;
}

static Boolean
geoVtPathPlanningTryTypeFallbackAfterMainFailure(const GeoVtPathPlanningInput_t *input,
                                                 const GeoVtPathPlanningRunParam_t *param,
                                                 const GeoVtPathPlanningOutput_t *failedMainOutput,
                                                 Integer startPosTyp,
                                                 Boolean isFrontStartType,
                                                 GeoVtPathPlanningOutput_t *output) {
  Boolean allowHybridFallback;
  Boolean fallbackOk;
  Boolean geometricOk;
  Boolean hybridOk;
  Integer fallbackGeomCandidateFlg;
  Integer fallbackGeomMode;
  Integer fallbackHybridCandidateFlg;
  Integer fallbackWinnerKind;
  Integer fallbackDecisionReason;
  Integer fallbackGeomStpN;
  Real fallbackGeomPathLen;
  Integer fallbackGeomSegNum;
  Integer fallbackGeomPlnChkCnt;
  Integer fallbackHybridStpN;
  Real fallbackHybridPathLen;
  Integer fallbackHybridSegNum;
  Integer fallbackHybridPlnChkCnt;
  GeoVtPathPlanningOutput_t geometricCandidate;
  GeoVtPathPlanningOutput_t hybridCandidate;
  GeoVtPathPlanningOutput_t bestCandidate;

  allowHybridFallback = (apaPlannerHybridResolveFusion(param->Base.NeedHybridAlgFusion) != 0);
  fallbackOk = false;
  hybridOk = false;
  fallbackGeomCandidateFlg = 0;
  fallbackHybridCandidateFlg = 0;
  fallbackGeomStpN = 0;
  fallbackGeomPathLen = 0.0f;
  fallbackGeomSegNum = 0;
  fallbackGeomPlnChkCnt = 0;
  fallbackHybridStpN = 0;
  fallbackHybridPathLen = 0.0f;
  fallbackHybridSegNum = 0;
  fallbackHybridPlnChkCnt = 0;
  geometricCandidate = (GeoVtPathPlanningOutput_t){0};
  hybridCandidate = (GeoVtPathPlanningOutput_t){0};
  bestCandidate = (GeoVtPathPlanningOutput_t){0};
  *output = *failedMainOutput;

  (void)fprintf(stderr,
                "[APA_VT_SEARCH] stage=FALLBACK start_type=%ld front=%d\n",
                (long)startPosTyp,
                (int)isFrontStartType);
  fallbackGeomMode = GEO_VT_FALLBACK_GEOM_MODE_SAFE;
  geometricOk =
      geoVtPathPlanningTryBestGeometricFallback(input, param, startPosTyp, &geometricCandidate);
  if (geometricOk) {
    fallbackGeomCandidateFlg = 1;
    bestCandidate = geometricCandidate;
    fallbackOk = true;
  }
  geoVtPathPlanningStoreFallbackDiagFromCandidate(geometricOk,
                                                  &geometricCandidate,
                                                  &fallbackGeomStpN,
                                                  &fallbackGeomPathLen,
                                                  &fallbackGeomSegNum,
                                                  &fallbackGeomPlnChkCnt);

  if (allowHybridFallback) {
    hybridOk = geoVtPathPlanningTryHybridFallback(input, param, &hybridCandidate);
    if (hybridOk) {
      fallbackHybridCandidateFlg = 1;
    }
    geoVtPathPlanningStoreFallbackDiagFromCandidate(hybridOk,
                                                    &hybridCandidate,
                                                    &fallbackHybridStpN,
                                                    &fallbackHybridPathLen,
                                                    &fallbackHybridSegNum,
                                                    &fallbackHybridPlnChkCnt);
    if (geoVtPathPlanningShouldReplaceCandidate(&bestCandidate, &hybridCandidate, param)) {
      bestCandidate = hybridCandidate;
    }
    fallbackOk = geoVtPathPlanningHasValidCandidate(&bestCandidate);
  }

  fallbackWinnerKind =
      geoVtPathPlanningResolveFallbackWinnerKind(geometricOk, hybridOk, &bestCandidate);
  fallbackDecisionReason = geoVtPathPlanningResolveFallbackDecisionReason(
      geometricOk, hybridOk, &geometricCandidate, &hybridCandidate, param);
  if (fallbackOk) {
    geoVtPathPlanningCommitCandidate(&bestCandidate, failedMainOutput->PlnChkCnt, output);
    if (output->PlannerCase != GEO_VT_PLANNER_CASE_HYBRID_FALLBACK) {
      output->PlannerCase = GEO_VT_PLANNER_CASE_GENERIC_FALLBACK;
    }
    geoVtPathPlanningLoadDiagLists(
        failedMainOutput->StpPthTypList, failedMainOutput->FailRsOut, output);
  }

  output->FallbackGeomCandidateFlg = fallbackGeomCandidateFlg;
  output->FallbackGeomMode = fallbackGeomMode;
  output->FallbackHybridCandidateFlg = fallbackHybridCandidateFlg;
  output->FallbackWinnerKind = fallbackWinnerKind;
  output->FallbackDecisionReason = fallbackDecisionReason;
  output->FallbackGeomStpN = fallbackGeomStpN;
  output->FallbackGeomPathLen = fallbackGeomPathLen;
  output->FallbackGeomSegNum = fallbackGeomSegNum;
  output->FallbackGeomPlnChkCnt = fallbackGeomPlnChkCnt;
  output->FallbackHybridStpN = fallbackHybridStpN;
  output->FallbackHybridPathLen = fallbackHybridPathLen;
  output->FallbackHybridSegNum = fallbackHybridSegNum;
  output->FallbackHybridPlnChkCnt = fallbackHybridPlnChkCnt;

  return fallbackOk;
}

/**
 * @brief VT垂直泊车完整适配器，统一接入位置分类、起点空间检测与 staged VT
 * planner
 * @en_name geoVtPathPlanningWithPolicy
 * @param[IN] GeoVtPathPlanningPolicy_t Policy 可选本次车位约束与候选验收
 * @cn_name VT路径规划
 * @type widget_module
 * @config cn_name=车位状态, en_name=SltState, desc=APA车位状态输入, unit=1,
 * type=Integer, range=[0,6], default=1
 * @config cn_name=成功车位最小半径, en_name=RminClassifiedSlot,
 * desc=sltsts为1或3时对齐MATLAB para(26)的最小转弯半径, unit=m,
 * type=Real, range=[1.0,10.0], default=5.5
 * @config cn_name=车宽, en_name=VehW, desc=车辆宽度, unit=m, type=Real,
 * range=[1.0,3.0], default=1.9
 * @config cn_name=前悬到参考点, en_name=VehLlF, desc=前悬到规划参考点长度,
 * unit=m, type=Real, range=[0.1,3.0], default=0.9
 * @config cn_name=最小前向安全长, en_name=VehLfMin, desc=最小前向安全长度,
 * unit=m, type=Real, range=[0.0,3.0], default=0.3
 * @config cn_name=后悬到参考点, en_name=VehLr, desc=后悬到规划参考点长度,
 * unit=m, type=Real, range=[0.1,3.0], default=0.98246
 * @config cn_name=回退空间, en_name=FallbackSpace,
 * desc=边界探测失败时的回退空间, unit=m, type=Real, range=[0.5,8.0],
 * default=4.1
 * @config cn_name=Hybrid融合开关, en_name=NeedHybridAlgFusion,
 * desc=是否允许在几何规划失败后进入HybridA*回退, unit=1, type=Integer,
 * range=[0,1], default=1
 * @param[IN] Real VehX 车辆X
 * @param[IN] Real VehY 车辆Y
 * @param[IN] Real VehTheta 车辆航向
 * @param[IN] Real TarX 目标X
 * @param[IN] Real TarY 目标Y
 * @param[IN] Real TarTheta 目标航向
 * @param[IN] Real Rmin 最小转弯半径
 * @param[IN] Real EntrPtX 入口点X
 * @param[IN] Real EntrPtY 入口点Y
 * @param[IN] Real SltPt1X 车位角点1X
 * @param[IN] Real SltPt1Y 车位角点1Y
 * @param[IN] Real SltPt2X 车位角点2X
 * @param[IN] Real SltPt2Y 车位角点2Y
 * @param[IN] Real SltPt3X 车位角点3X
 * @param[IN] Real SltPt3Y 车位角点3Y
 * @param[IN] Real SltPt4X 车位角点4X
 * @param[IN] Real SltPt4Y 车位角点4Y
 * @param[IN] Integer SltEntr 车位入口边编号
 * @param[IN] Integer PrkType 泊车方向1左2右
 * @param[IN] Integer SurdType 邻车类型0两侧有车1仅右后方2仅左前方3无邻车4未知
 * @param[IN] Real AgrOfst 可用空间修正量
 * @param[IN] Integer BoundaryLineNum 规划边界线数量
 * @param[IN] Real[64] BoundaryLineX1 边界线起点X数组
 * @param[IN] Real[64] BoundaryLineY1 边界线起点Y数组
 * @param[IN] Real[64] BoundaryLineX2 边界线终点X数组
 * @param[IN] Real[64] BoundaryLineY2 边界线终点Y数组
 * @param[IN] Integer VirtualBoundaryLineNum 虚拟边界线数量
 * @param[IN] Real[64] VirtualBoundaryP1X 虚拟边界点1X
 * @param[IN] Real[64] VirtualBoundaryP1Y 虚拟边界点1Y
 * @param[IN] Real[64] VirtualBoundaryP2X 虚拟边界点2X
 * @param[IN] Real[64] VirtualBoundaryP2Y 虚拟边界点2Y
 * @param[IN] Integer BoundarySepPtNum 真实离散点数量
 * @param[IN] Real[128] BoundarySepPtX 真实离散点X
 * @param[IN] Real[128] BoundarySepPtY 真实离散点Y
 * @param[IN] Integer StopperPtNum 挡停点数量
 * @param[IN] Real[128] StopperPtX 挡停点X
 * @param[IN] Real[128] StopperPtY 挡停点Y
 * @param[OUT] Integer PlanFlg 规划成功标志
 * @param[OUT] Integer SegNum 输出段数量
 * @param[OUT] Real PathLen 输出路径总长
 * @param[OUT] Integer[9] SegGoDir 各段行驶方向
 * @param[OUT] Integer[9] SegTrnDir 各段转弯方向
 * @param[OUT] Real[9] SegCrv 各段曲率
 * @param[OUT] Real[9] SegTht 各段转角
 * @param[OUT] Real[9] SegLen 各段长度
 * @param[OUT] Real[9] SegStX 各段起点X
 * @param[OUT] Real[9] SegStY 各段起点Y
 * @param[OUT] Real[9] SegStTh 各段起点航向
 * @param[OUT] Real[9] SegCcX 各段圆心X
 * @param[OUT] Real[9] SegCcY 各段圆心Y
 * @param[OUT] Integer StartPosTyp 起点相对位置类型
 * @param[OUT] Real StartRelaX 起点相对X
 * @param[OUT] Real StartRelaY 起点相对Y
 * @param[OUT] Real StartRelaTheta 起点相对航向
 * @param[OUT] Real ProbeLeftSpace 左侧探测空间
 * @param[OUT] Real ProbeRightSpace 右侧探测空间
 * @param[OUT] Real ResolvedEntrPtX 最终使用的入口点X
 * @param[OUT] Real ResolvedEntrPtY 最终使用的入口点Y
 * @param[OUT] Integer EntrDgrPtFlg 入口危险点标志
 * @param[OUT] Integer EntrCheckCnt 入口点检测检查次数
 * @param[OUT] Integer PlnChkCnt 规划检查次数
 * @param[OUT] Integer StpN 换向次数
 * @param[OUT] Integer PlannerCase 规划分支1主解2前方起点专用分支3通用回退4HybridA*回退
 * @param[OUT] Integer FallbackGeomCandidateFlg 当前分支几何回退候选存在标志
 * @param[OUT] Integer FallbackGeomMode 当前分支几何回退模式0未尝试1安全几何2原始几何
 * @param[OUT] Integer FallbackHybridCandidateFlg 当前分支Hybrid回退候选存在标志
 * @param[OUT] Integer FallbackWinnerKind 当前分支回退赢家0无1几何2Hybrid
 * @param[OUT] Integer FallbackDecisionReason 当前分支回退决策原因0无1仅有效性2换向次数3路径长度4段数5检查次数6原生路径质量
 * @param[OUT] Integer FallbackGeomStpN 当前分支几何回退候选换向次数
 * @param[OUT] Real FallbackGeomPathLen 当前分支几何回退候选路径总长
 * @param[OUT] Integer FallbackGeomSegNum 当前分支几何回退候选段数
 * @param[OUT] Integer FallbackGeomPlnChkCnt 当前分支几何回退候选检查次数
 * @param[OUT] Integer FallbackHybridStpN 当前分支Hybrid回退候选换向次数
 * @param[OUT] Real FallbackHybridPathLen 当前分支Hybrid回退候选路径总长
 * @param[OUT] Integer FallbackHybridSegNum 当前分支Hybrid回退候选段数
 * @param[OUT] Integer FallbackHybridPlnChkCnt 当前分支Hybrid回退候选检查次数
 * @param[OUT] Integer StpPthTypLN 路径类型序列长度
 * @param[OUT] Integer StpPthTyp0 路径类型序列首项
 * @param[OUT] Integer StpPthTyp1 路径类型序列次项
 * @param[OUT] Integer[30] StpPthTypList 路径类型序列
 * @param[OUT] Integer FailRs0 失败原因首项
 * @param[OUT] Integer FailRs1 失败原因次项
 * @param[OUT] Integer[30] FailRsOut 失败原因序列
 * @granularity composite
 * @tag_level1 泊车规划
 * @tag_level2 路径规划
 * @source matlab_rewrite geo_VT_PathPlanning.m
 * @version 1.1
 * @date 2026-03-09
 * @author aide
 */
void geoVtPathPlanningWithPolicy(const GeoVtPathPlanningInput_t *input,
                                 const GeoVtPathPlanningParam_t *configuration,
                                 const GeoVtPathPlanningPolicy_t *policy,
                                 GeoVtPathPlanningOutput_t *output) {
  GeoVtPathPlanningRunParam_t runParam;
  const GeoVtPathPlanningRunParam_t *param;
  GeoVtPathPlanningInput_t plannerInput;
  GeoVtEntrPtDngPtInput_t entrIn;
  GeoVtEntrPtDngPtOutput_t entrOut;
  GeoStPtSpcVtOutput_t spcOut;
  GeoVtPathPlanningMainLoopWork_t mainLoopWork;
  GeoVtPathPlanningOutput_t mainLoopOutput;
  Boolean hasEntrSceneInput;
  Boolean targetPrecheckSafe;
  Boolean initialOffsetOk;
  Boolean mainLoopPlanOk;
  Integer entrCheckCnt;
  Integer startPosTyp;
  Boolean isFrontStartType;

  runParam.Base = *configuration;
  runParam.Policy = policy;
  param = &runParam;

  plannerInput = *input;
  plannerInput.Rmin = geoVtPathPlanningResolveRmin(input, param);
  geoVtPathPlanningMainLoopWorkInit(&mainLoopWork, &plannerInput);
  entrIn = (GeoVtEntrPtDngPtInput_t){0};
  entrOut = (GeoVtEntrPtDngPtOutput_t){0};
  spcOut = (GeoStPtSpcVtOutput_t){0};
  mainLoopOutput = (GeoVtPathPlanningOutput_t){0};
  *output = (GeoVtPathPlanningOutput_t){0};
  entrCheckCnt = 0;

  targetPrecheckSafe = geoVtPathPlanningRunTargetPrecheck(&plannerInput, param, output);
  if (targetPrecheckSafe) {
    initialOffsetOk =
        geoVtPathPlanningMainLoopRunInitialOffset(&plannerInput, param, &mainLoopWork, &spcOut);
    if (initialOffsetOk) {
      plannerInput.VehX = mainLoopWork.PlnEdPtX;
      plannerInput.VehY = mainLoopWork.PlnEdPtY;
      plannerInput.VehTheta = mainLoopWork.PlnEdPtTheta;
    }

    hasEntrSceneInput = geoVtPathPlanningHasEntrSceneInput(&plannerInput);
    if (hasEntrSceneInput) {
      geoVtPathPlanningBuildEntrSceneInput(&plannerInput, &entrIn);
      geoVtEntrPtDngPt(&entrIn, &entrOut);
      plannerInput.EntrPtX = entrOut.EntrPlanX;
      plannerInput.EntrPtY = entrOut.EntrPlanY;
      entrCheckCnt = entrOut.CheckCount;
    }

    if (plannerInput.EntrPtX == 0.0f && plannerInput.EntrPtY == 0.0f) {
      plannerInput.EntrPtX = 0.5f * (plannerInput.SltPt1X + plannerInput.SltPt2X);
      plannerInput.EntrPtY = 0.5f * (plannerInput.SltPt1Y + plannerInput.SltPt2Y);
    }

    mainLoopWork.Pt1X = plannerInput.EntrPtX;
    mainLoopWork.Pt1Y = plannerInput.EntrPtY;
    geoVtPathPlanningMainLoopWriteTerminalReverseSegment(&plannerInput, &mainLoopWork);
    mainLoopPlanOk =
        geoVtPathPlanningMainLoopRunPlanLoop(&plannerInput, param, &mainLoopWork, &mainLoopOutput);
    if (mainLoopPlanOk) {
      mainLoopOutput.PlannerCase = GEO_VT_PLANNER_CASE_MAIN;
      mainLoopPlanOk = geoVtPathPlanningAcceptPolicyCandidate(param, &mainLoopOutput);
    }
    *output = mainLoopOutput;
    if (!mainLoopPlanOk) {
      output->PlanFlg = 0;
      mainLoopOutput.PlanFlg = 0;
      startPosTyp = mainLoopOutput.StartPosTyp;
      if (startPosTyp == 0) {
        startPosTyp = mainLoopOutput.StpPthTypList[0];
      }

      GeoVtPathPlanningInput_t fallbackInput = plannerInput;
      fallbackInput.VehX = input->VehX;
      fallbackInput.VehY = input->VehY;
      fallbackInput.VehTheta = input->VehTheta;
      isFrontStartType = geoVtPathPlanningIsFrontStartType(startPosTyp);
      (void)geoVtPathPlanningTryTypeFallbackAfterMainFailure(
          &fallbackInput, param, &mainLoopOutput, startPosTyp, isFrontStartType, output);
      if (output->PlanFlg == 0) {
        output->PlannerCase = GEO_VT_PLANNER_CASE_NONE;
      }
    }

    if (output->StartPosTyp == 0) {
      output->StartPosTyp = output->StpPthTypList[0];
    }
    output->ProbeLeftSpace = spcOut.LeftSpace;
    output->ProbeRightSpace = spcOut.RightSpace;
    output->ResolvedEntrPtX = plannerInput.EntrPtX;
    output->ResolvedEntrPtY = plannerInput.EntrPtY;
    output->EntrDgrPtFlg = entrOut.DgrPtFlg;
    output->EntrCheckCnt = entrCheckCnt;
  }
  if (!targetPrecheckSafe) {

    (void)geoVtPathPlanningTryTypeFallbackAfterMainFailure(
        &plannerInput, param, output, 0, false, output);
  }
  (void)fprintf(stderr,
                "[APA_VT_SEARCH] stage=RESULT target_safe=%d start_type=%ld fail_reason=%ld "
                "geometric_candidate=%ld hybrid_candidate=%ld plan_ok=%ld planner_case=%ld\n",
                (int)targetPrecheckSafe,
                (long)output->StartPosTyp,
                (long)output->FailRs0,
                (long)output->FallbackGeomCandidateFlg,
                (long)output->FallbackHybridCandidateFlg,
                (long)output->PlanFlg,
                (long)output->PlannerCase);
}

/**
 * @brief Run the source-reference VT planner without an APA publication policy.
 * @en_name geoVtPathPlanning
 * @cn_name VT几何路径规划
 * @type module
 * @param[IN] GeoVtPathPlanningInput_t Input 规划输入
 * @param[IN] GeoVtPathPlanningParam_t Param 几何与车辆参数
 * @param[OUT] GeoVtPathPlanningOutput_t Output 分段候选及来源诊断
 * @granularity composite
 * @tag_level1 规划
 * @source matlab_rewrite
 * @version 1.2
 * @date 2026-09-30
 * @author aide
 */
void geoVtPathPlanning(const GeoVtPathPlanningInput_t *input,
                       const GeoVtPathPlanningParam_t *param,
                       GeoVtPathPlanningOutput_t *output) {
  geoVtPathPlanningWithPolicy(input, param, (const GeoVtPathPlanningPolicy_t *)0, output);
}
