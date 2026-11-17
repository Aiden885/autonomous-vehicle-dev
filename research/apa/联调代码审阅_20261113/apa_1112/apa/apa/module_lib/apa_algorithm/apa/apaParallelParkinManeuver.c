#include "apaParallelParkinManeuver.h"

static Boolean frontCornerOutside(const ApaSlot_t *slot,
                                  const ApaPlannerVehConfig_t *vehicle,
                                  const Real pose[3],
                                  Boolean right) {
  const Integer near = slot->EnterEdge % 4;
  const Real lateral = (right ? -0.5f : 0.5f) * vehicle->W;
  const Real x = pose[0] + vehicle->LF * cosf(pose[2]) - lateral * sinf(pose[2]);
  const Real y = pose[1] + vehicle->LF * sinf(pose[2]) + lateral * cosf(pose[2]);
  const Real dx = x - slot->SlotPts[2 * near], dy = y - slot->SlotPts[2 * near + 1];
  const Real normal = slot->TarPos[2] + (right ? -0.5f : 0.5f) * APA_PI;
  const Real length = hypotf(dx, dy);
  return length > APA_PLANNER_HYBRID_EPS &&
         (dx * cosf(normal) + dy * sinf(normal)) / length < cosf(94.0f * APA_PI / 180.0f);
}

static Real scanPrimitive(const ApaSlot_t *slot,
                          Boolean right,
                          ApaPlannerHybridCollisionContext_t *collision,
                          const Real start[3],
                          Integer direction,
                          Real curvature,
                          Real limit,
                          Boolean checkVertex,
                          Boolean requireWhole,
                          Boolean *vertexOut,
                          const char *stage,
                          Boolean *blocked) {
  const ApaPlannerConfig_t *config =
      apaPlannerGlobalGetPlanner(collision->CollisionContext.PlannerGlobal);
  ApaHybridNode_t from, to;
  apaNodeInit(&from);
  apaNodeInit(&to);
  from.X = start[0];
  from.Y = start[1];
  from.Theta = start[2];
  Real safe = 0.0f;
  *vertexOut = false;
  if (blocked)
    *blocked = false;
  const Integer samples = (Integer)ceilf(limit / 0.03f) + 1;
  for (Integer i = 0; i < samples; ++i) {
    const Real distance = 0.04f + 0.03f * (Real)i;
    if (!apaMathLess(distance, limit))
      break;
    apaPlannerHybridEvalPrimitivePose(
        direction, curvature, distance, start[0], start[1], start[2], &to.X, &to.Y, &to.Theta);
    to.D = (Real)direction * (distance - safe);
    to.Delta = atanf(config->Veh.WB * curvature);
    if (apaPlannerHybridCollisionCheck(&from, &to, collision)) {
      if (blocked && !*vertexOut)
        *blocked = true;
      if (stage && !*vertexOut)
        fprintf(stderr,
                "[APA_HT_MANEUVER_STEP] stage=%s safe=%.3f blocked=(%.3f,%.3f,%.5f)\n",
                stage,
                (double)safe,
                (double)to.X,
                (double)to.Y,
                (double)to.Theta);
      return requireWhole ? 0.0f : safe;
    }
    safe = distance;
    if (checkVertex) {
      const Real pose[3] = {to.X, to.Y, to.Theta};
      if (frontCornerOutside(slot, &config->Veh, pose, right)) {
        *vertexOut = true;
      }
      if (*vertexOut && !apaMathLess(safe, 0.43f)) {
        break;
      }
    }
    from = to;
  }
  return safe;
}

static void lastPose(const ApaHybridAStarMotionPath_t *motion, Real pose[3]) {
  const Integer i = motion->Path.PathN - 1;
  pose[0] = motion->Path.X[i];
  pose[1] = motion->Path.Y[i];
  pose[2] = motion->Path.Tht[i];
}

static Boolean appendPrimitive(ApaHybridAStarMotionPath_t *motion,
                               Integer direction,
                               Real curvature,
                               Real length) {
  ApaPath_t *path = &motion->Path;
  const Integer i = path->PathN - 1;
  if (i < 0 || path->PathN >= APA_PATH_MAX_POINTS || !isfinite(length) ||
      length <= APA_PLANNER_HYBRID_EPS)
    return false;
  apaPlannerHybridEvalPrimitivePose(direction,
                                    curvature,
                                    length,
                                    path->X[i],
                                    path->Y[i],
                                    path->Tht[i],
                                    &path->X[i + 1],
                                    &path->Y[i + 1],
                                    &path->Tht[i + 1]);
  path->OdoStep[i] = (Real)direction * length;
  motion->Curvature[i] = curvature;
  path->PathN++;
  path->OdoStep[i + 1] = motion->Curvature[i + 1] = 0.0f;
  path->ValidFlag = true;
  apaPathResetFlags(path);
  return true;
}

Boolean apaParallelParkinManeuver(const ApaSlot_t *slot,
                                  const Real vehicle[3],
                                  ApaPlannerHybridCollisionContext_t *collision,
                                  ApaHybridAStarMotionPath_t *tail,
                                  ApaParallelParkinManeuverDiag_t *diag) {
  *tail = (ApaHybridAStarMotionPath_t){0};
  apaPathInit(&tail->Path);
  *diag = (ApaParallelParkinManeuverDiag_t){0};
  diag->Reason = "invalid_input";
  if (!slot || !collision || !collision->CollisionContext.PlannerGlobal || slot->EnterEdge < 1 ||
      slot->EnterEdge > 4)
    return false;
  const ApaPlannerConfig_t *config =
      apaPlannerGlobalGetPlanner(collision->CollisionContext.PlannerGlobal);
  if (!isfinite(config->Veh.MinCircle) || config->Veh.MinCircle <= 0.0f)
    return false;

  Boolean right = false;
  for (Integer offset = 1; offset <= 2; ++offset) {
    const Integer corner = (slot->EnterEdge + offset) % 4;
    const Real dx = slot->SlotPts[2 * corner] - slot->TarPos[0];
    const Real dy = slot->SlotPts[2 * corner + 1] - slot->TarPos[1];
    if (-sinf(slot->TarPos[2]) * dx + cosf(slot->TarPos[2]) * dy < 0.0f)
      right = true;
  }
  const Real curvature = (right ? 1.0f : -1.0f) / config->Veh.MinCircle;
  const Integer savedModel = collision->CollisionContext.PlannerGlobal->Planner.Conf.CollisionModel;
  const Integer savedNativeModel = collision->SceneTemplate.CollisionModel;
  collision->CollisionContext.PlannerGlobal->Planner.Conf.CollisionModel =
      APA_COLLISION_MODEL_EIGHT_EDGES;
  collision->SceneTemplate.CollisionModel = APA_COLLISION_MODEL_EIGHT_EDGES;

  ApaHybridAStarMotionPath_t outward = {0};
  apaPathInit(&outward.Path);
  outward.Path.PathN = 1;
  outward.Path.PathType = APA_PATH_TYPE_ALL;
  outward.Path.PathSource = APA_PATH_SOURCE_SEARCH;
  outward.Path.ErrorType = APA_FAULT_NO_ERR;
  outward.Path.X[0] = slot->TarPos[0];
  outward.Path.Y[0] = slot->TarPos[1];
  outward.Path.Tht[0] = slot->TarPos[2];
  Real pose[3];
  lastPose(&outward, pose);
  ApaHybridNode_t anchor;
  apaNodeInit(&anchor);
  anchor.X = pose[0];
  anchor.Y = pose[1];
  anchor.Theta = pose[2];
  Boolean vertex = false, unused = false, success = false;
  if (apaPlannerHybridCollisionCheck(&anchor, &anchor, collision)) {
    diag->Reason = "target_blocked";
    goto done;
  }
  const Real initial = scanPrimitive(
      slot, right, collision, pose, -1, 0.0f, 2.0f, false, false, &unused, "initial_reverse", 0);

  if (!apaMathLess(initial, config->Conf.PathLenMin) &&
      !appendPrimitive(&outward, -1, 0.0f, initial)) {
    diag->Reason = "path_capacity";
    goto done;
  }
  for (Integer loop = 1; loop <= 6 && !vertex; ++loop) {
    diag->Loops = loop;
    lastPose(&outward, pose);
    Boolean forwardBlocked = false;
    const Real forward = scanPrimitive(slot,
                                       right,
                                       collision,
                                       pose,
                                       1,
                                       curvature,
                                       4.0f,
                                       true,
                                       false,
                                       &vertex,
                                       "forward",
                                       &forwardBlocked);
    const Boolean forwardOk = !apaMathLess(forward, config->Conf.PathLenMin);
    if (!forwardOk)
      vertex = false;
    if (forwardOk && !appendPrimitive(&outward, 1, curvature, forward)) {
      diag->Reason = "path_capacity";
      goto done;
    }
    Real backward = 0.0f;
    Boolean backwardOk = false;
    if (!vertex) {
      lastPose(&outward, pose);
      backward = scanPrimitive(
          slot, right, collision, pose, -1, -curvature, 4.0f, false, false, &unused, "backward", 0);
      backwardOk = !apaMathLess(backward, config->Conf.PathLenMin);
      if (backwardOk && !appendPrimitive(&outward, -1, -curvature, backward)) {
        diag->Reason = "path_capacity";
        goto done;
      }
    }
    fprintf(stderr,
            "[APA_HT_MANEUVER_LOOP] loop=%ld forward=%.3f backward=%.3f vertex_out=%d\n",
            (long)loop,
            (double)forward,
            (double)backward,
            (int)vertex);
    if (!forwardOk && !backwardOk) {
      diag->Reason = "no_valid_maneuver";
      goto done;
    }

    if (!vertex && forwardBlocked && forwardOk && !backwardOk) {
      diag->Reason = "reverse_blocked";
      goto done;
    }
  }
  if (!vertex) {
    diag->Reason = "six_loop_limit";
    goto done;
  }

  if (!apaSlotCalcVehInSlotFlag(vehicle, slot->SlotPts, 0, &config->Veh)) {
    ApaHybridAStarMotionPath_t adjusted = outward;
    collision->CollisionContext.PlannerGlobal->Planner.Conf.CollisionModel =
        APA_COLLISION_MODEL_FOUR_EDGES;
    collision->SceneTemplate.CollisionModel = APA_COLLISION_MODEL_FOUR_EDGES;
    Boolean adjustedOk = true;
    for (Integer i = 0; i < 2 && adjustedOk; ++i) {
      lastPose(&adjusted, pose);
      const Real curve = (i == 0 ? curvature : -curvature) * 0.1f;
      const Real length = scanPrimitive(slot,
                                        right,
                                        collision,
                                        pose,
                                        1,
                                        curve,
                                        i == 0 ? 1.5f : 1.0f,
                                        false,
                                        true,
                                        &unused,
                                        "exit_adjustment",
                                        0);
      adjustedOk = !apaMathLess(length, config->Conf.PathLenMin) &&
                   appendPrimitive(&adjusted, 1, curve, length);
    }
    if (adjustedOk) {
      outward = adjusted;
      diag->Adjusted = true;
    }
  }
  *tail = outward;
  apaPlannerHybridReverseMotion(tail);
  diag->Reason = "generated";
  success = true;
done:
  collision->CollisionContext.PlannerGlobal->Planner.Conf.CollisionModel = savedModel;
  collision->SceneTemplate.CollisionModel = savedNativeModel;
  return success;
}

Boolean apaParallelParkinManeuverInRegion(const ApaSlot_t *slot,
                                          const Real vehicle[3],
                                          Real tolerance,
                                          ApaPlannerHybridCollisionContext_t *collision,
                                          ApaHybridAStarMotionPath_t *tail,
                                          ApaParallelParkinManeuverDiag_t *diag) {
  for (Integer i = 0; i < 5; ++i)
    if (apaParallelParkinManeuverRegionCandidate(
            slot, vehicle, tolerance, i, collision, tail, diag))
      return true;
  return false;
}

Boolean apaParallelParkinManeuverRegionCandidate(const ApaSlot_t *slot,
                                                 const Real vehicle[3],
                                                 Real tolerance,
                                                 Integer index,
                                                 ApaPlannerHybridCollisionContext_t *collision,
                                                 ApaHybridAStarMotionPath_t *tail,
                                                 ApaParallelParkinManeuverDiag_t *diag) {
  static const Real scales[5] = {0.0f, -0.5f, 0.5f, -1.0f, 1.0f};
  if (!slot || index < 0 || index >= 5 || !isfinite(tolerance) || tolerance < 0.0f ||
      (index && tolerance == 0.0f))
    return false;
  ApaSlot_t shifted = *slot;
  const Real distance = scales[index] * fminf(0.2f, 0.8f * tolerance);
  shifted.TarPos[0] += distance * cosf(slot->TarPos[2]);
  shifted.TarPos[1] += distance * sinf(slot->TarPos[2]);
  if (!apaParallelParkinManeuver(&shifted, vehicle, collision, tail, diag))
    return false;
  fprintf(stderr,
          "[APA_HT_MANEUVER_GOAL] index=%ld offset=%.6f tolerance=%.6f\n",
          (long)index,
          (double)distance,
          (double)tolerance);
  return true;
}
