#ifndef APA_TARGET_SLOT_CONSTRAINT_H
#define APA_TARGET_SLOT_CONSTRAINT_H

#include "apaTargetSlotConstraintTypes.h"

Boolean apaTargetSlotConstraintPoseIsClear(const ApaTargetSlotConstraint_t *constraint,
                                           Real poseX,
                                           Real poseY,
                                           Real poseTheta,
                                           Integer *collisionEdge);
void apaTargetSlotConstraintCheckMotion(const ApaTargetSlotConstraint_t *constraint,
                                        const ApaTargetSlotMotion_t *motion,
                                        ApaTargetSlotMotionCheck_t *output);

#endif
