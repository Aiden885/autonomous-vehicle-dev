#ifndef APA_PLANNING_SCENE_H
#define APA_PLANNING_SCENE_H

#include "vehicleCollisionCheckTypes.h"
#include "apaVehicleDefaults.h"
#include "apaPlanningSlotGuidance.h"
#include <math.h>

/* Preferred room for optional seeds/initial search, not final admission. */
#define APA_PLANNING_PREFERRED_CLEARANCE_M 0.10f

/* Worker-local query; no IPC pointers. Planning margins are totals;
 * TrackingReserve supplements other check modes. */
typedef Boolean (*ApaPlanningSceneCheck_f)(const VehicleCollisionCheckInput_t *pose,
                                           void *userData);
/* Heading-independent disk overlap for guidance only; cannot admit motion. */
typedef Boolean (*ApaPlanningSceneDiskCheck_f)(Real x, Real y, Real radius, void *userData);
/* Worker-local execution check for batches of geometrically admitted candidates. */
typedef Boolean (*ApaPlanningExecutionCheck_f)(Integer count,
                                               const Real *x,
                                               const Real *y,
                                               const Real *yaw,
                                               const Real *odometer,
                                               void *userData);

typedef struct {
  ApaPlanningSceneCheck_f CheckPose;
  void *UserData;
  Real TrackingReserve; /* Additional envelope reserve, m; zero disables. */
  ApaPlanningSceneDiskCheck_f CheckDisk;
  ApaPlanningExecutionCheck_f CheckExecution;
  void *ExecutionData;
} ApaPlanningSceneQuery_t;

static inline Real apaPlanningPhysicalExtraReserve(const VehicleCollisionCheckInput_t *input,
                                                   Real reserve) {
  return input->CheckType == VCC_CHECK_TYPE_PLANNING ? 0.0f : reserve;
}

/* Full body octagon; caller supplies resolved margins and constraint sources. */
static inline VehicleCollisionCheckInput_t
apaCollisionBodyEnvelope(const VehicleCollisionCheckInput_t *input, Real reserve) {
  VehicleCollisionCheckInput_t body = *input;
  body.CollisionModel = VCC_COLLISION_MODEL_EIGHT_EDGES;
  body.UseAutoMargin = false;
  body.SlotPtsValid = body.VehInSlotFlag = false;
  body.FrontSafeMargin = body.RearSafeMargin = body.SideSafeMargin = 0.0f;
  body.WhlBseLongMargin = body.WhlBseLatMargin = 0.0f;
  body.VehLF += reserve;
  body.VehLB += reserve;
  body.VehLL += 2.0f * reserve;
  body.VehW += 2.0f * reserve;
  return body;
}

static inline void apaPlanningSceneCollisionCheck(const VehicleCollisionCheckInput_t *input,
                                                  const ApaPlanningSceneQuery_t *scene,
                                                  VehicleCollisionCheckOutput_t *output) {
  vehicleCollisionCheck(input, output);
  if (!output->IsCollision && scene && scene->CheckPose &&
      scene->CheckPose(input, scene->UserData)) {
    output->IsCollision = true;
    output->CollisionType = VCC_COLLISION_BOUNDARY;

    output->CollisionIndex = 0;
  }
  if (!output->IsCollision && scene) {
    const Real reserve = apaPlanningPhysicalExtraReserve(input, scene->TrackingReserve);
    if (!(reserve >= 0.0f && reserve <= 0.2f)) {
      output->IsCollision = true;
      output->CollisionType = VCC_COLLISION_BOUNDARY;
      output->CollisionIndex = 0;
    } else if (reserve > 0.0f) {
      VehicleCollisionCheckInput_t body = apaCollisionBodyEnvelope(input, reserve);
      Integer physical;
      body.VehNearSlotFlag = false;
      for (physical = 0; physical < 2 && !output->IsCollision; ++physical) {
        VehicleCollisionCheckOutput_t checked;
        if (physical) {
          body = apaCollisionBodyEnvelope(input, apaPlanningPhysicalExtraReserve(input, reserve));
          body.VehNearSlotFlag = false;
          body.FrontSafeMargin = output->SafeMargin1stBound[0];
          body.RearSafeMargin = output->SafeMargin1stBound[1];
          body.SideSafeMargin = output->SafeMargin1stBound[2];
        }
        vehicleCollisionCheck(&body, &checked);
        if (checked.IsCollision) {
          *output = checked;
        } else if (scene->CheckPose && scene->CheckPose(&body, scene->UserData)) {
          output->IsCollision = true;
          output->CollisionType = VCC_COLLISION_BOUNDARY;
          output->CollisionIndex = 0;
        }
      }
    }
  }
}

/* Eight expanded-envelope soft bands; side/bottom lines retain preference.
 * Entrance/foreign paint is filtered only from cost. No Euclidean-distance metric. */
static inline Boolean
apaPlanningSceneClearanceBandIsClear(const VehicleCollisionCheckInput_t *input,
                                     const ApaPlanningSceneQuery_t *scene,
                                     const Real margins[3],
                                     Real extra) {
  VehicleCollisionCheckInput_t body = *input;
  VehicleCollisionCheckOutput_t checked;
  body.UseAutoMargin = false;
  body.FrontSafeMargin = margins[0] + extra;
  body.RearSafeMargin = margins[1] + extra;
  body.SideSafeMargin = margins[2] + extra;
  if (input->CheckType == VCC_CHECK_TYPE_PLANNING && !input->ParkingOutFlag &&
      input->UseAutoMargin) {

    body.WhlBseLongMargin = body.WhlBseLatMargin = APA_PARKIN_PHYSICAL_MARGIN_M + extra;
  } else {
    body.WhlBseLongMargin += extra;
    body.WhlBseLatMargin += extra;
  }
  apaPlanningSceneCollisionCheck(&body, scene, &checked);
  return !checked.IsCollision;
}

static inline Real apaPlanningSceneObstaclePenalty(const VehicleCollisionCheckInput_t *input,
                                                   const ApaPlanningSceneQuery_t *scene) {
  const Integer bandCount = 8;
  const Boolean parkin = input->CheckType == VCC_CHECK_TYPE_PLANNING && !input->ParkingOutFlag;
  const Real maximumPenalty = parkin ? 0.30f : 2.0f;
  VehicleCollisionCheckInput_t softInput;

  if (parkin && input->SlotPtsValid && input->VirtualBoundaryLineNum > 0) {
    softInput = *input;
    apaPlanningSlotSoftVirtualLines(input, &softInput);
    input = &softInput;
  }
  Real margins[3];
  vehicleCollisionCheckResolveBodyMargins(input, margins);

  const Real extraRange = parkin
                              ? 0.15f + fmaxf(0.0f,
                                              APA_PLANNING_PREFERRED_CLEARANCE_M -
                                                  fminf(margins[0], fminf(margins[1], margins[2])))
                              : 0.30f;

  if (input->CheckType == VCC_CHECK_TYPE_PLANNING && !input->ParkingOutFlag &&
      (input->CollisionModel == VCC_COLLISION_MODEL_FOUR_EDGES ||
       input->CollisionModel == VCC_COLLISION_MODEL_EIGHT_EDGES)) {
    if (apaPlanningSceneClearanceBandIsClear(input, scene, margins, extraRange))
      return 0.0f;
    Integer clear = 0, blocked = bandCount;
    while (blocked - clear > 1) {
      const Integer middle = (clear + blocked) / 2;
      if (apaPlanningSceneClearanceBandIsClear(
              input, scene, margins, extraRange * (Real)middle / (Real)bandCount))
        clear = middle;
      else
        blocked = middle;
    }
    const Real deficit = 1.0f - (Real)clear / (Real)bandCount;
    return maximumPenalty * deficit * deficit;
  }
  for (Integer band = bandCount; band > 0; --band) {
    const Real extra = extraRange * (Real)band / (Real)bandCount;
    if (apaPlanningSceneClearanceBandIsClear(input, scene, margins, extra)) {
      const Real deficit = 1.0f - (Real)band / (Real)bandCount;
      return maximumPenalty * deficit * deficit;
    }
  }
  return maximumPenalty;
}

/* Shared per-metre soft cost for primitives, RS connections and candidate ranking. */
static inline Real apaPlanningSceneClearancePenalty(const VehicleCollisionCheckInput_t *input,
                                                    const ApaPlanningSceneQuery_t *scene) {
  return apaPlanningSceneObstaclePenalty(input, scene) + apaPlanningSlotAlignmentPenalty(input);
}

/* First-feasible guidance omits alignment and in-slot paint costs; quality
 * labels and final ranking retain the full soft cost. */
static inline Real apaPlanningSceneFeasibilityPenalty(const VehicleCollisionCheckInput_t *input,
                                                      const ApaPlanningSceneQuery_t *scene) {
  if (input->CheckType == VCC_CHECK_TYPE_PLANNING && !input->ParkingOutFlag &&
      input->SlotPtsValid && input->VirtualBoundaryLineNum > 0 &&
      apaPlanningSlotBodyCenterIsInside(input)) {
    VehicleCollisionCheckInput_t relaxed = *input;
    relaxed.VirtualBoundaryLineNum = 0;
    return apaPlanningSceneObstaclePenalty(&relaxed, scene);
  }
  return apaPlanningSceneObstaclePenalty(input, scene);
}

#endif
