#ifndef APATHIRDBOUNDARYTYPES_H
#define APATHIRDBOUNDARYTYPES_H

#include "apaThirdBoundaryUtils.h"
#include "apaThirdBoundary.h"
#include "rectCollisionCheckForBoundaryTypes.h"

typedef struct {
  Real VehPos[3];
  Integer SlotClass;
  Integer SurroundingType;
  Integer ParkingType;
  ApaSlot_t SlotList[4];
  Integer SlotCount;
  Integer TarSlotPtr;
  VehicleCollisionCheckInput_t Obs;
  Integer SlotSubStatus;
  Boolean IsNormalScene;
  Boolean UseSideExtendFlag;
  ApaPlannerVehConfig_t VehConfig;
} ApaThirdBoundaryGetBoundaryInput_t;

typedef struct {
  ApaThirdBoundaryLineSet_t OutLines;
  Boolean Success;
} ApaThirdBoundaryGetBoundaryOutput_t;

/* Canonical structured entry. */
void apaThirdBoundaryGetBoundaryAide(const ApaThirdBoundaryGetBoundaryInput_t *input,
                                     ApaThirdBoundaryGetBoundaryOutput_t *output);

void apaThirdBoundaryGetBoundary_aide(const ApaThirdBoundaryGetBoundaryInput_t *input,
                                      ApaThirdBoundaryGetBoundaryOutput_t *output);

#endif
