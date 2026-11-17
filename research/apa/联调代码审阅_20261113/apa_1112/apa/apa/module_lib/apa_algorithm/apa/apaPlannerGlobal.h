/**
 * @file apaPlannerGlobal.h
 * @brief Explicit planner-global state wrapper for get_planner_global.m.
 *
 * Source:
 *   - get_planner_global.m
 *
 * Date: 2026-04-14
 */
#ifndef APA_PLANNER_GLOBAL_H
#define APA_PLANNER_GLOBAL_H

#include "apaPlannerConfig.h"
#include "baseTypes.h"

/* Zero-init literal for safe first-use lazy initialization. */
#define APA_PLANNER_GLOBAL_STATE_INIT                                                              \
  { 0 }

typedef struct {
  Boolean Initialized;
  ApaPlannerConfig_t Planner;
} ApaPlannerGlobalState_t;

static inline Boolean apaPlannerGlobalIsInitialized(const ApaPlannerGlobalState_t *state) {
  return state->Initialized;
}

static inline void apaPlannerGlobalInit(ApaPlannerGlobalState_t *state) {
  apaPlannerConfigInitDefault(&state->Planner);
  state->Initialized = true;
}

static inline void apaPlannerGlobalReset(ApaPlannerGlobalState_t *state) {
  apaPlannerGlobalInit(state);
}

static inline void apaPlannerGlobalEnsureInit(ApaPlannerGlobalState_t *state) {
  if (!state->Initialized) {
    apaPlannerGlobalInit(state);
  }
}

static inline const ApaPlannerConfig_t *apaPlannerGlobalGetPlanner(ApaPlannerGlobalState_t *state) {
  apaPlannerGlobalEnsureInit(state);
  return &state->Planner;
}

static inline const ApaPlannerMapConfig_t *apaPlannerGlobalGetMap(ApaPlannerGlobalState_t *state) {
  apaPlannerGlobalEnsureInit(state);
  return &state->Planner.Map;
}

static inline const ApaPlannerVehConfig_t *apaPlannerGlobalGetVeh(ApaPlannerGlobalState_t *state) {
  apaPlannerGlobalEnsureInit(state);
  return &state->Planner.Veh;
}

static inline const ApaPlannerConfConfig_t *
apaPlannerGlobalGetConf(ApaPlannerGlobalState_t *state) {
  apaPlannerGlobalEnsureInit(state);
  return &state->Planner.Conf;
}

static inline const ApaPlannerSlotConfig_t *
apaPlannerGlobalGetSlot(ApaPlannerGlobalState_t *state) {
  apaPlannerGlobalEnsureInit(state);
  return &state->Planner.Slot;
}

static inline void apaPlannerGlobalSetSbCost(ApaPlannerGlobalState_t *state, Real value) {
  apaPlannerGlobalEnsureInit(state);
  state->Planner.Map.SbCost = value;
}

static inline void apaPlannerGlobalSetEnableYawGainPenalty(ApaPlannerGlobalState_t *state,
                                                           Boolean value) {
  Integer enableValue;

  apaPlannerGlobalEnsureInit(state);
  if (value) {
    enableValue = 1;
  } else {
    enableValue = 0;
  }
  state->Planner.Conf.EnableYawGainPenalty = enableValue;
}

static inline void apaPlannerGlobalSetXyGridResolution(ApaPlannerGlobalState_t *state, Real value) {
  apaPlannerGlobalEnsureInit(state);
  state->Planner.Map.XyGridResolution = value;
}

static inline void apaPlannerGlobalSetYawGridResolution(ApaPlannerGlobalState_t *state,
                                                        Real value) {
  apaPlannerGlobalEnsureInit(state);
  state->Planner.Map.YawGridResolution = value;
  state->Planner.Map.MapYawPixNum =
      (Integer)roundf((state->Planner.Map.MaxYaw - state->Planner.Map.MinYaw) / value);
}

static inline void apaPlannerGlobalSetStepLengthTimes(ApaPlannerGlobalState_t *state, Real value) {
  Real stepLength;

  apaPlannerGlobalEnsureInit(state);
  stepLength = value * state->Planner.Map.XyGridResolution;
  state->Planner.Map.MotionStepLength = stepLength;
}

static inline void apaPlannerGlobalSetSteerCost(ApaPlannerGlobalState_t *state, Real value) {
  apaPlannerGlobalEnsureInit(state);
  state->Planner.Map.SteerCost = value;
}

static inline void apaPlannerGlobalSetPlanningCalPowerMaxPerFrame(ApaPlannerGlobalState_t *state,
                                                                  Integer value) {
  apaPlannerGlobalEnsureInit(state);
  state->Planner.Conf.PlanningCalPowerMaxPerFrame = value;
}

static inline void apaPlannerGlobalSetPlanningCalPowerMaxPerSlot(ApaPlannerGlobalState_t *state,
                                                                 Integer value) {
  apaPlannerGlobalEnsureInit(state);
  state->Planner.Conf.PlanningCalPowerMaxPerSlot = value;
}

static inline void apaPlannerGlobalSetHeuristicValueType(ApaPlannerGlobalState_t *state,
                                                         Integer value) {
  apaPlannerGlobalEnsureInit(state);
  state->Planner.Conf.HeuristicValueType = value;
}

static inline void apaPlannerGlobalSetNeedHybridAlgFusion(ApaPlannerGlobalState_t *state,
                                                          Integer value) {
  apaPlannerGlobalEnsureInit(state);
  state->Planner.Conf.NeedHybridAlgFusion = value;
}

static inline void apaPlannerGlobalSetNSteer(ApaPlannerGlobalState_t *state, Real value) {
  apaPlannerGlobalEnsureInit(state);
  state->Planner.Map.NSteer = value;
}

static inline void apaPlannerGlobalSetReverseTheta(ApaPlannerGlobalState_t *state, Real value) {
  apaPlannerGlobalEnsureInit(state);
  state->Planner.Slot.ReverseTheta = value;
}

static inline void apaPlannerGlobalSetReverseFlag(ApaPlannerGlobalState_t *state, Integer value) {
  Integer reverseFlagValue;

  apaPlannerGlobalEnsureInit(state);
  if (value != 0) {
    reverseFlagValue = 1;
  } else {
    reverseFlagValue = 0;
  }
  state->Planner.Slot.ReverseFlag = reverseFlagValue;
}

static inline void apaPlannerGlobalSetCollisionModel(ApaPlannerGlobalState_t *state,
                                                     Integer value) {
  apaPlannerGlobalEnsureInit(state);
  state->Planner.Conf.CollisionModel = value;
}

static inline void apaPlannerGlobalSetLongAllowableErrMin(ApaPlannerGlobalState_t *state,
                                                          Real value) {
  apaPlannerGlobalEnsureInit(state);
  state->Planner.Conf.LongAllowableErrMin = value;
}

static inline void apaPlannerGlobalSetLongAllowableErrMax(ApaPlannerGlobalState_t *state,
                                                          Real value) {
  apaPlannerGlobalEnsureInit(state);
  state->Planner.Conf.LongAllowableErrMax = value;
}

static inline void apaPlannerGlobalSetLatAllowableErrMin(ApaPlannerGlobalState_t *state,
                                                         Real value) {
  apaPlannerGlobalEnsureInit(state);
  state->Planner.Conf.LatAllowableErrMin = value;
}

static inline void apaPlannerGlobalSetLatAllowableErrMax(ApaPlannerGlobalState_t *state,
                                                         Real value) {
  apaPlannerGlobalEnsureInit(state);
  state->Planner.Conf.LatAllowableErrMax = value;
}

static inline void apaPlannerGlobalSetYawAllowableErrMin(ApaPlannerGlobalState_t *state,
                                                         Real value) {
  apaPlannerGlobalEnsureInit(state);
  state->Planner.Conf.YawAllowableErrMin = value;
}

static inline void apaPlannerGlobalSetYawAllowableErrMax(ApaPlannerGlobalState_t *state,
                                                         Real value) {
  apaPlannerGlobalEnsureInit(state);
  state->Planner.Conf.YawAllowableErrMax = value;
}

static inline void apaPlannerGlobalSetUseMoreCalUnits(ApaPlannerGlobalState_t *state,
                                                      Boolean value) {
  Boolean useMoreCalUnits;

  apaPlannerGlobalEnsureInit(state);
  useMoreCalUnits = value;
  state->Planner.Conf.UseMoreCalUnits = useMoreCalUnits;
}

static inline void apaPlannerGlobalSetAllowableErrorWindow(ApaPlannerGlobalState_t *state,
                                                           Real longErrMin,
                                                           Real longErrMax,
                                                           Real latErrMin,
                                                           Real latErrMax,
                                                           Real yawErrMin,
                                                           Real yawErrMax) {
  apaPlannerGlobalSetLongAllowableErrMin(state, longErrMin);
  apaPlannerGlobalSetLongAllowableErrMax(state, longErrMax);
  apaPlannerGlobalSetLatAllowableErrMin(state, latErrMin);
  apaPlannerGlobalSetLatAllowableErrMax(state, latErrMax);
  apaPlannerGlobalSetYawAllowableErrMin(state, yawErrMin);
  apaPlannerGlobalSetYawAllowableErrMax(state, yawErrMax);
}

#endif
