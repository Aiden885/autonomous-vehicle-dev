#ifndef APA_TRACKING_POLICY_H
#define APA_TRACKING_POLICY_H

#include "apaVehicleProfileGenerated.h"

/* Additional envelope reserve for non-planning checks, m. */
#define APA_PLANNING_TRACKING_RESERVE_M 0.05f

/* Clearance restoration requires collision-free, single-direction motion. */
#define APA_CLEARANCE_RECOVERY_MAX_LENGTH_M 0.8
#define APA_CLEARANCE_RECOVERY_MAX_SPEED_MPS 0.15
#define APA_CLEARANCE_RECOVERY_MAX_DURATION_SEC 15.0
/* Preferred recovery join gap; constrained scenes may use a normally admissible fallback. */
#define APA_CLEARANCE_RECOVERY_TARGET_GAP_M 0.15

/* Inside the tracking-error gate, static/map uses an 8 mm physical floor so the
 * 4 cm physical planning margin leaves >=3.2 cm straight-edge (>5 cm corner)
 * tracking tolerance. Zero-height painted virtual lines do not trigger mid-run
 * emergency stops; final goal overlap is still checked. */
#define APA_TRACKING_VIRTUAL_CLEARANCE_M 0.0
#define APA_TRACKING_STATIC_MAP_CLEARANCE_M 0.008
/* Tracking gate covers cusp capture (20 cm / 6 deg). */
#define APA_TRACKING_REFERENCE_ERROR_M 0.25
#define APA_TRACKING_HEADING_ERROR_RAD 0.13962634015954636
/* Allowances are relative to the frozen origin, never per sample. */
#define APA_CLEARANCE_RECOVERY_GAP_SWING_M 0.02
#define APA_CLEARANCE_RECOVERY_MIN_GAP_M 0.0

#endif
