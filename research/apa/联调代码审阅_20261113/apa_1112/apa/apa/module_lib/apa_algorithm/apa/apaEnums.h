/**
 * @file apaEnums.h
 * @brief APA 规划子系统全量枚举定义
 *
 * 来源：MATLAB PlanningSubFcn/Enum*.m (27个)
 * 规范：typedef enum + Integer 底层类型，命名 APA_ENUM_VALUE 大写下划线
 *
 * @date 2026-04-13
 */
#ifndef APA_ENUMS_H
#define APA_ENUMS_H

#include "baseTypes.h"

/** @brief 碰撞检测类型 — 对应 EnumCollisionCheckType.m */
typedef enum {
  APA_COLLISION_CHECK_PLANNING = 0,
  APA_COLLISION_CHECK_GENERAL = 1,
  APA_COLLISION_CHECK_DYNAMIC = 2,
  APA_COLLISION_CHECK_STATIC = 3
} ApaCollisionCheckType_e;

/** @brief 碰撞检测需求 — 对应 EnumCollisionDemand.m */
typedef enum {
  APA_COLLISION_DEMAND_SIMPLIFY = 0,
  APA_COLLISION_DEMAND_ALL = 1
} ApaCollisionDemand_e;

/** @brief 碰撞检测模型 — 对应 EnumCollisionModel.m */
typedef enum {
  APA_COLLISION_MODEL_FOUR_EDGES = 0,
  APA_COLLISION_MODEL_EIGHT_EDGES = 1
} ApaCollisionModel_e;

/** @brief 碰撞风险 — 对应 EnumCollisionRisk.m */
typedef enum {
  APA_COLLISION_RISK_UNKNOWN = 0,
  APA_COLLISION_RISK_EXIST = 1,
  APA_COLLISION_RISK_NOT_EXIST = 2
} ApaCollisionRisk_e;

/** @brief 泊车方向 — 对应 EnumParkingDirec.m */
typedef enum {
  APA_PARKING_DIREC_INVALID = 0,
  APA_PARKING_DIREC_FRONT = 1,
  APA_PARKING_DIREC_REAR = 2,
  APA_PARKING_DIREC_OUT = 3
} ApaParkingDirec_e;

/** @brief 泊车系统主状态 — 对应 EnumParkingSysMainState.m */
typedef enum {
  APA_SYS_STATE_INVALID = 0,
  APA_SYS_STATE_SEARCHING = 1,
  APA_SYS_STATE_PARKING = 2
} ApaParkingSysMainState_e;

/** @brief 泊车类型(左/右) — 对应 EnumParkingType.m */
typedef enum {
  APA_PARKING_TYPE_MEANINGLESS = 0,
  APA_PARKING_TYPE_LEFT = 1,
  APA_PARKING_TYPE_RIGHT = 2,
  APA_PARKING_TYPE_UNKNOWN = 3
} ApaParkingType_e;

/** @brief 档位 — 对应 EnumGearPos.m */
typedef enum {
  APA_GEAR_POS_N = 0,
  APA_GEAR_POS_D = 1,
  APA_GEAR_POS_R = 2,
  APA_GEAR_POS_P = 3
} ApaGearPos_e;

/** @brief Map signed path direction to the requested drive/reverse gear. */
static inline Integer apaGearPosForGoDir(Integer goDir) {
  if (goDir < 0) {
    return APA_GEAR_POS_R;
  }
  return APA_GEAR_POS_D;
}

/** @brief Require the actual drive/reverse gear to match the path direction. */
static inline Boolean apaGearPosMatchesGoDir(Integer gearPos, Integer goDir) {
  Integer gearReq0;
  Boolean condMatched0;

  gearReq0 = apaGearPosForGoDir(goDir);
  condMatched0 = (gearPos == gearReq0);
  return condMatched0;
}

/** @brief 期望方向标志 — 对应 EnumExpDirec.m */
typedef enum {
  APA_EXP_DIREC_NONE = 0,
  APA_EXP_DIREC_FORCE_FORWARD = 1,
  APA_EXP_DIREC_FORCE_BACKWARD = 2,
  APA_EXP_DIREC_SUGGEST_FORWARD = 3,
  APA_EXP_DIREC_SUGGEST_BACKWARD = 4
} ApaExpDirec_e;

/** @brief 车位分类 — 对应 EnumSlotClass.m */
typedef enum {
  APA_SLOT_CLASS_VERTICAL = 0,
  APA_SLOT_CLASS_PARALLEL = 1,
  APA_SLOT_CLASS_UNKNOWN = 2
} ApaSlotClass_e;

/** @brief 车位状态 — 对应 EnumSlotStatus.m */
typedef enum {
  APA_SLOT_STATUS_INVALID = 0,
  APA_SLOT_STATUS_TARGET_SLOT = 1,
  APA_SLOT_STATUS_WAIT_PLAN = 2,
  APA_SLOT_STATUS_FINISH_PLAN = 3,
  APA_SLOT_STATUS_FAIL_PLAN = 4,
  APA_SLOT_STATUS_UNCHECKED = 5,
  APA_SLOT_STATUS_TARGET_SLOT_REPLAN = 6,
  APA_SLOT_STATUS_TARGET_FINISH_REPLAN = 7,
  APA_SLOT_STATUS_TARGET_FAIL_REPLAN = 8,
  APA_SLOT_STATUS_RESERVE_1 = 9,
  APA_SLOT_STATUS_RESERVE_2 = 10,
  APA_SLOT_STATUS_RESERVE_3 = 11,
  APA_SLOT_STATUS_STOP_PLAN = 12,
  APA_SLOT_STATUS_RESERVE_4 = 13,
  APA_SLOT_STATUS_RESERVE_5 = 14,
  APA_SLOT_STATUS_RESERVE_6 = 15,
  APA_SLOT_STATUS_RESERVE_7 = 16,
  APA_SLOT_STATUS_RESERVE_8 = 17,
  APA_SLOT_STATUS_RESERVE_9 = 18,
  APA_SLOT_STATUS_RESERVE_10 = 19,
  APA_SLOT_STATUS_RESERVE_11 = 20
} ApaSlotStatus_e;

/** @brief 车位子状态 — 对应 EnumSlotSubStatus.m */
typedef enum {
  APA_SLOT_SUB_END_NONE = 0,
  APA_SLOT_SUB_END_FIRST = 1,
  APA_SLOT_SUB_END_SECOND = 2,
  APA_SLOT_SUB_END_THIRD = 3
} ApaSlotSubStatus_e;

/** @brief 路径比较结果 — 对应 EnumPathComp.m */
typedef enum {
  APA_PATH_COMP_UNKNOWN = 0,
  APA_PATH_COMP_ORIGIN_PATH_BETTER = 1,
  APA_PATH_COMP_NEW_PATH_BETTER = 2
} ApaPathComp_e;

/** @brief 路径列表类型 — 对应 EnumPathListType.m */
typedef enum {
  APA_PATH_LIST_INVALID = 0,
  APA_PATH_LIST_GEO = 1,
  APA_PATH_LIST_SEARCH = 2,
  APA_PATH_LIST_FUSE = 3,
  APA_PATH_LIST_MEM = 4
} ApaPathListType_e;

/** @brief 路径来源 — 对应 EnumPathSource.m */
typedef enum {
  APA_PATH_SOURCE_INVALID = 0,
  APA_PATH_SOURCE_GEO = 1,
  APA_PATH_SOURCE_SEARCH = 2
} ApaPathSource_e;

/** @brief 路径类型 — 对应 EnumPathType.m */
typedef enum {
  APA_PATH_TYPE_UNKNOWN = 0,
  APA_PATH_TYPE_TERMINATION = 1,
  APA_PATH_TYPE_ALL = 2
} ApaPathType_e;

/** @brief 拟合类型 — 对应 EnumPolyfitType.m */
typedef enum {
  APA_POLYFIT_INVALID = 0,
  APA_POLYFIT_DOMAIN_X = 1,
  APA_POLYFIT_DOMAIN_Y = 2
} ApaPolyfitType_e;

/** @brief 规划状态 — 对应 EnumPlanningStatus.m */
typedef enum {
  APA_PLANNING_STATUS_INVALID = 0,
  APA_PLANNING_STATUS_WAITING = 1,
  APA_PLANNING_STATUS_SUCCEED = 2,
  APA_PLANNING_STATUS_FAIL = 3,
  APA_PLANNING_STATUS_PARKING_ZONE_INVALID = 4
} ApaPlanningStatus_e;

/** @brief 规划故障码 — 对应 EnumPathPlanFault.m (57个值) */
typedef enum {
  APA_FAULT_NO_ERR = 0,
  APA_FAULT_FUSION_BOUND_INVALID = 1,
  APA_FAULT_SLOT_INVALID = 2,
  APA_FAULT_COLLISION_AT_START = 3,
  APA_FAULT_COLLISION_AT_TARGET_SLOT = 4,
  APA_FAULT_START_GRID_ILLEGAL = 5,
  APA_FAULT_GOAL_GRID_ILLEGAL = 6,
  APA_FAULT_HYBRID_OPEN_RUN_OUT = 7,
  APA_FAULT_HYBRID_CLOSE_RUN_OUT = 8,
  APA_FAULT_HYBRID_OPEN_EMPTY = 9,
  APA_FAULT_HYBRID_PATH_OUT_OF_RANGE = 10,
  APA_FAULT_HYBRID_PATH_RES_TOO_LARGE = 11,
  APA_FAULT_NOT_TARGET_SLOT = 12,
  APA_FAULT_EXCEED_MAX_LEN_IN_PLANNER_CORE = 13,
  APA_FAULT_INVALID_APA_STATE = 14,
  APA_FAULT_LESS_CAL_POWER = 15,
  APA_FAULT_PARKING_CLEAR_CACHE = 16,
  APA_FAULT_PLANNER_NOT_ACCEPT_THE_SLOT = 17,
  APA_FAULT_COLLISION_AT_START_2ND_BOUND = 18,
  APA_FAULT_COLLISION_AT_START_3RD_BOUND = 19,
  APA_FAULT_COLLISION_AT_TARGET_SLOT_2ND_BOUND = 20,
  APA_FAULT_COLLISION_AT_TARGET_SLOT_3RD_BOUND = 21,
  APA_FAULT_SEG_NUM_OVERMANY = 22,
  APA_FAULT_CS_INCONSISTENT = 23,
  APA_FAULT_APA_DIRECTION_CHANGED = 24,
  APA_FAULT_CURVATURE_EXCEEDING_LIMIT = 25,
  APA_FAULT_COLLISION_RISK_AT_NEW_OBS = 26,
  APA_FAULT_TAR_POS_ERR_EXCEEDING_LIMIT = 27,
  APA_FAULT_VEH_AND_SLOT_TOO_FAR = 28,
  APA_FAULT_STOPPED_SLOT_WAIT_REPLAN = 29,
  APA_FAULT_HYBRID_UNKNOWN_ERR = 30,
  APA_FAULT_UNKNOWN_ERR = 31,
  APA_FAULT_PKING_OUT_UNENABLE = 32,
  APA_FAULT_NAVI_RESET_ABNORMAL = 33,
  APA_FAULT_SLOT_AVALIABLE_BUT_PATH_INVALID = 34,
  APA_FAULT_RESERVE_1 = 35,
  APA_FAULT_MASSIVE_CAL_POWER_LACK = 36,
  APA_FAULT_NO_PATHS_TO_CHANGE_DIRECTION = 37,
  APA_FAULT_PATH_NOT_MATCH_TARPOS = 38,
  APA_FAULT_POST_PROCESS_CS_INCONSISTENT = 39,
  APA_FAULT_SPT_CLEAR_SLOT_STATUS = 40,
  APA_FAULT_PATH_STATUS_ERR = 41,
  APA_FAULT_PATH_NOT_MATCH_TARPOS_IN_LOCAL = 42,
  APA_FAULT_PATH_LEN_INVALID = 43,
  APA_FAULT_CLOSE_PARENT_INVALID = 44,
  APA_FAULT_PATH_END_ZERO = 45,
  APA_FAULT_PATH_END_NOT_IN_SLOT = 46,
  APA_FAULT_PATH_END_FAR_FROM_TARPOS = 47,
  APA_FAULT_GEO_EQUAL_COE_INVALID = 48,
  APA_FAULT_GEO_PATH_REVERSE_COLLISION = 49,
  APA_FAULT_GEO_SEG_NUM_OVERMANY = 50,
  APA_FAULT_GEO_PATH_OVERLAP = 51,
  APA_FAULT_GEO_ENVIRO_COMPLEX = 52,
  APA_FAULT_GEO_INHIBITION = 53,
  APA_FAULT_PATH_N_EXCEED_TO_ADD = 54,
  APA_FAULT_SEARCH_PATH_END_NOT_EQUAL_FB_PATH_START = 55,
  APA_FAULT_PARALLEL_REPLAN_IN_SLOT_FAIL = 56
} ApaPathPlanFault_e;

/** @brief Hybrid A* 故障码 — 对应 EnumHybridAStarFault.m */
typedef enum {
  APA_HYBRID_FAULT_NO_ERR = 0,
  APA_HYBRID_FAULT_ILLEGAL_START_GRID = 1,
  APA_HYBRID_FAULT_ILLEGAL_GOAL_GRID = 2,
  APA_HYBRID_FAULT_HYBRID_OPEN_RUN_OUT = 3,
  APA_HYBRID_FAULT_HYBRID_CLOSE_RUN_OUT = 4,
  APA_HYBRID_FAULT_HYBRID_OPEN_EMPTY = 5,
  APA_HYBRID_FAULT_PATH_OUT_OF_RANGE = 6,
  APA_HYBRID_FAULT_PATH_RESOLUTION_TOO_LARGE = 7,
  APA_HYBRID_FAULT_EXCEED_CAL_POWER = 8,
  APA_HYBRID_FAULT_PARENT_INVALID = 9,
  APA_HYBRID_FAULT_UNKNOWN_ERR = 255
} ApaHybridAStarFault_e;

/** @brief 重规划请求类型 — 对应 EnumReplanReqType.m */
typedef enum {
  APA_REPLAN_NONE = 0,
  APA_REPLAN_GET_BETTER_GOAL_POS = 1,
  APA_REPLAN_REDUCE_CONTRL_ERR = 2,
  APA_REPLAN_REDUCE_PATH_SEG = 3,
  APA_REPLAN_AVOID_COLLISION = 4,
  APA_REPLAN_AVOID_PUNCH_STOPPER = 5,
  APA_REPLAN_SLOT_BOTTOM_REPLAN = 6
} ApaReplanReqType_e;

/** @brief 第三类重规划类型 — 对应 EnumThirdReplanType.m */
typedef enum {
  APA_THIRD_REPLAN_NONE = 0,
  APA_THIRD_REPLAN_OBS_COLLISION = 1,
  APA_THIRD_REPLAN_SIXTH_REPLAN_CONVERT = 2,
  APA_THIRD_REPLAN_FIFTH_REPLAN_CONVERT = 3,
  APA_THIRD_REPLAN_MEB_CONVERT = 4,
  APA_THIRD_REPLAN_BLOCK_CONVERT = 5,
  APA_THIRD_REPLAN_TAR_POS_DEVIATES_FROM_PATH_END = 6,
  APA_THIRD_REPLAN_NEAR_OBS_COLLISION = 7,
  APA_THIRD_REPLAN_PARKING_OUT_END_NEAR_OBS = 8
} ApaThirdReplanType_e;

/** @brief 制动需求标志 — 对应 EnumNeedBrakeFlag.m */
typedef enum {
  APA_BRAKE_NONE = 0,
  APA_BRAKE_AND_STOP = 1,
  APA_BRAKE_AND_PULL_REPLAN = 2,
  APA_BRAKE_GO_BACK = 3,
  APA_BRAKE_QUIT_CONTROL = 4
} ApaNeedBrakeFlag_e;

/** @brief 边界组合类型 — 对应 EnumBoundComposeType.m */
typedef enum {
  APA_BOUND_COMPOSE_NONE = 0,
  APA_BOUND_COMPOSE_DETECTED = 1,
  APA_BOUND_COMPOSE_UNDETECTED = 2,
  APA_BOUND_COMPOSE_BOTH = 3
} ApaBoundComposeType_e;

/** @brief 融合边界选择需求 — 对应 EnumSelectFusedBoundDemand.m */
typedef enum {
  APA_FUSED_BOUND_ALL = 0,
  APA_FUSED_BOUND_DYNAMIC = 1,
  APA_FUSED_BOUND_STATIC = 2,
  APA_FUSED_BOUND_OTHERS = 3
} ApaSelectFusedBoundDemand_e;

/** @brief 环绕类型 — 对应 EnumSurroundingType.m */
typedef enum {
  APA_SURROUND_OBS_EXIST_BOTH = 0,
  APA_SURROUND_OBS_EXIST_RIGHT_REAR_ONLY = 1,
  APA_SURROUND_OBS_EXIST_LEFT_FRONT_ONLY = 2,
  APA_SURROUND_OBS_EXIST_NONE = 3,
  APA_SURROUND_UNKNOWN = 4
} ApaSurroundingType_e;

/** @brief 环视相机标志 — 对应 EnumSurroundingViewCameraFlag.m */
typedef enum {
  APA_CAMERA_INVALID = 0,
  APA_CAMERA_FRONT = 1,
  APA_CAMERA_BACK = 2
} ApaSurroundingViewCameraFlag_e;

/* ══════════ MATLAB enum class-name flag helpers ══════════ */

/**
 * @brief MATLAB Enum*.m addClassNameToEnumNames 公共语义
 *
 * MATLAB 枚举类的静态方法均返回 true，用于生成带类名前缀的枚举名。
 * C runtime 只需要保留该调试/导出契约，不改变枚举值本身。
 */
static inline Boolean apaEnumAddClassNameToEnumNames(void) {
  Boolean retVal0;

  retVal0 = true;
  return retVal0;
}

/** @brief 对应 EnumBoundComposeType.addClassNameToEnumNames */
static inline Boolean apaEnumBoundComposeTypeAddClassNameToEnumNames(void) {
  Boolean retVal0;

  retVal0 = apaEnumAddClassNameToEnumNames();
  return retVal0;
}

/** @brief 对应 EnumCollisionCheckType.addClassNameToEnumNames */
static inline Boolean apaEnumCollisionCheckTypeAddClassNameToEnumNames(void) {
  Boolean retVal0;

  retVal0 = apaEnumAddClassNameToEnumNames();
  return retVal0;
}

/** @brief 对应 EnumCollisionDemand.addClassNameToEnumNames */
static inline Boolean apaEnumCollisionDemandAddClassNameToEnumNames(void) {
  Boolean retVal0;

  retVal0 = apaEnumAddClassNameToEnumNames();
  return retVal0;
}

/** @brief 对应 EnumCollisionModel.addClassNameToEnumNames */
static inline Boolean apaEnumCollisionModelAddClassNameToEnumNames(void) {
  Boolean retVal0;

  retVal0 = apaEnumAddClassNameToEnumNames();
  return retVal0;
}

/** @brief 对应 EnumCollisionRisk.addClassNameToEnumNames */
static inline Boolean apaEnumCollisionRiskAddClassNameToEnumNames(void) {
  Boolean retVal0;

  retVal0 = apaEnumAddClassNameToEnumNames();
  return retVal0;
}

/** @brief 对应 EnumExpDirec.addClassNameToEnumNames */
static inline Boolean apaEnumExpDirecAddClassNameToEnumNames(void) {
  Boolean retVal0;

  retVal0 = apaEnumAddClassNameToEnumNames();
  return retVal0;
}

/** @brief 对应 EnumGearPos.addClassNameToEnumNames */
static inline Boolean apaEnumGearPosAddClassNameToEnumNames(void) {
  Boolean retVal0;

  retVal0 = apaEnumAddClassNameToEnumNames();
  return retVal0;
}

/** @brief 对应 EnumHybridAStarFault.addClassNameToEnumNames */
static inline Boolean apaEnumHybridAStarFaultAddClassNameToEnumNames(void) {
  Boolean retVal0;

  retVal0 = apaEnumAddClassNameToEnumNames();
  return retVal0;
}

/** @brief 对应 EnumNeedBrakeFlag.addClassNameToEnumNames */
static inline Boolean apaEnumNeedBrakeFlagAddClassNameToEnumNames(void) {
  Boolean retVal0;

  retVal0 = apaEnumAddClassNameToEnumNames();
  return retVal0;
}

/** @brief 对应 EnumParkingDirec.addClassNameToEnumNames */
static inline Boolean apaEnumParkingDirecAddClassNameToEnumNames(void) {
  Boolean retVal0;

  retVal0 = apaEnumAddClassNameToEnumNames();
  return retVal0;
}

/** @brief 对应 EnumParkingSysMainState.addClassNameToEnumNames */
static inline Boolean apaEnumParkingSysMainStateAddClassNameToEnumNames(void) {
  Boolean retVal0;

  retVal0 = apaEnumAddClassNameToEnumNames();
  return retVal0;
}

/** @brief 对应 EnumParkingType.addClassNameToEnumNames */
static inline Boolean apaEnumParkingTypeAddClassNameToEnumNames(void) {
  Boolean retVal0;

  retVal0 = apaEnumAddClassNameToEnumNames();
  return retVal0;
}

/** @brief 对应 EnumPathComp.addClassNameToEnumNames */
static inline Boolean apaEnumPathCompAddClassNameToEnumNames(void) {
  Boolean retVal0;

  retVal0 = apaEnumAddClassNameToEnumNames();
  return retVal0;
}

/** @brief 对应 EnumPathListType.addClassNameToEnumNames */
static inline Boolean apaEnumPathListTypeAddClassNameToEnumNames(void) {
  Boolean retVal0;

  retVal0 = apaEnumAddClassNameToEnumNames();
  return retVal0;
}

/** @brief 对应 EnumPathPlanFault.addClassNameToEnumNames */
static inline Boolean apaEnumPathPlanFaultAddClassNameToEnumNames(void) {
  Boolean retVal0;

  retVal0 = apaEnumAddClassNameToEnumNames();
  return retVal0;
}

/** @brief 对应 EnumPathSource.addClassNameToEnumNames */
static inline Boolean apaEnumPathSourceAddClassNameToEnumNames(void) {
  Boolean retVal0;

  retVal0 = apaEnumAddClassNameToEnumNames();
  return retVal0;
}

/** @brief 对应 EnumPathType.addClassNameToEnumNames */
static inline Boolean apaEnumPathTypeAddClassNameToEnumNames(void) {
  Boolean retVal0;

  retVal0 = apaEnumAddClassNameToEnumNames();
  return retVal0;
}

/** @brief 对应 EnumPlanningStatus.addClassNameToEnumNames */
static inline Boolean apaEnumPlanningStatusAddClassNameToEnumNames(void) {
  Boolean retVal0;

  retVal0 = apaEnumAddClassNameToEnumNames();
  return retVal0;
}

/** @brief 对应 EnumPolyfitType.addClassNameToEnumNames */
static inline Boolean apaEnumPolyfitTypeAddClassNameToEnumNames(void) {
  Boolean retVal0;

  retVal0 = apaEnumAddClassNameToEnumNames();
  return retVal0;
}

/** @brief 对应 EnumReplanReqType.addClassNameToEnumNames */
static inline Boolean apaEnumReplanReqTypeAddClassNameToEnumNames(void) {
  Boolean retVal0;

  retVal0 = apaEnumAddClassNameToEnumNames();
  return retVal0;
}

/** @brief 对应 EnumSelectFusedBoundDemand.addClassNameToEnumNames */
static inline Boolean apaEnumSelectFusedBoundDemandAddClassNameToEnumNames(void) {
  Boolean retVal0;

  retVal0 = apaEnumAddClassNameToEnumNames();
  return retVal0;
}

/** @brief 对应 EnumSlotClass.addClassNameToEnumNames */
static inline Boolean apaEnumSlotClassAddClassNameToEnumNames(void) {
  Boolean retVal0;

  retVal0 = apaEnumAddClassNameToEnumNames();
  return retVal0;
}

/** @brief 对应 EnumSlotStatus.addClassNameToEnumNames */
static inline Boolean apaEnumSlotStatusAddClassNameToEnumNames(void) {
  Boolean retVal0;

  retVal0 = apaEnumAddClassNameToEnumNames();
  return retVal0;
}

/** @brief 对应 EnumSlotSubStatus.addClassNameToEnumNames */
static inline Boolean apaEnumSlotSubStatusAddClassNameToEnumNames(void) {
  Boolean retVal0;

  retVal0 = apaEnumAddClassNameToEnumNames();
  return retVal0;
}

/** @brief 对应 EnumSurroundingType.addClassNameToEnumNames */
static inline Boolean apaEnumSurroundingTypeAddClassNameToEnumNames(void) {
  Boolean retVal0;

  retVal0 = apaEnumAddClassNameToEnumNames();
  return retVal0;
}

/** @brief 对应 EnumSurroundingViewCameraFlag.addClassNameToEnumNames */
static inline Boolean apaEnumSurroundingViewCameraFlagAddClassNameToEnumNames(void) {
  Boolean retVal0;

  retVal0 = apaEnumAddClassNameToEnumNames();
  return retVal0;
}

/** @brief 对应 EnumThirdReplanType.addClassNameToEnumNames */
static inline Boolean apaEnumThirdReplanTypeAddClassNameToEnumNames(void) {
  Boolean retVal0;

  retVal0 = apaEnumAddClassNameToEnumNames();
  return retVal0;
}

#endif
