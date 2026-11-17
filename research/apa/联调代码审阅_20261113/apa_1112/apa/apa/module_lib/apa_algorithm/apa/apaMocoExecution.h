#ifndef APA_MOCO_EXECUTION_H
#define APA_MOCO_EXECUTION_H
#include "baseTypes.h"

/* Keep the published 0..6 phase values stable. */
enum {
  APA_MOCO_NOACT = 0,
  APA_MOCO_INITIAL = 1,
  APA_MOCO_GEAR_CHECK = 2,
  APA_MOCO_TRACK = 3,
  APA_MOCO_STOP_WAIT = 4,
  APA_MOCO_SWITCH = 5,
  APA_MOCO_DONE = 6,
  APA_MOCO_DECEL = 7,
  APA_MOCO_CREEP = 8,
  APA_MOCO_FAULT = 9
};
enum {
  APA_MOCO_STOP_NONE = 0,
  APA_MOCO_STOP_CUSP = 1,
  APA_MOCO_STOP_GOAL = 2,
  APA_MOCO_STOP_INVALID = 3,
  APA_MOCO_STOP_REFERENCE = 4,
  APA_MOCO_STOP_ENDPOINT = 5,
  APA_MOCO_STOP_GEAR_TIMEOUT = 6,
  APA_MOCO_STOP_STALLED = 7,
  APA_MOCO_STOP_GOAL_BOUNDARY = 8,
  APA_MOCO_STOP_OVERSPEED = 9
};
static inline Boolean apaMocoMotionPhase(Integer phase) {
  return phase == APA_MOCO_TRACK || phase == APA_MOCO_DECEL || phase == APA_MOCO_CREEP;
}
#endif
