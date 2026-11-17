#include "apaMocoPathPointTransferV2Types.h"
#include "apaCoordUtils.h"
#include <math.h>

static Real apaMocoWeightedCurvature(const ApaMocoPathPointTransferV2Input_t *input,
                                     Real start,
                                     Real end,
                                     Real fallback) {
  Real prefix = 0.0f;
  Real curvature = 0.0f;
  Real weight = 0.0f;
  for (Integer i = 0; i + 1 < input->LocalPtCount && prefix < end; ++i) {
    const Real length = hypotf(input->LocalPathX[i + 1] - input->LocalPathX[i],
                               input->LocalPathY[i + 1] - input->LocalPathY[i]);
    const Real overlap = fminf(prefix + length, end) - fmaxf(prefix, start);
    const Real odometer = input->LocalPathOdo[i];
    if (overlap > 0.0f && fabsf(odometer) > 1.0e-5f) {
      const Real yawChange =
          apaCoordWrapToHalfOpenPi(input->LocalPathYaw[i + 1] - input->LocalPathYaw[i]);
      curvature += overlap * yawChange / odometer;
      weight += overlap;
    }
    prefix += length;
  }
  return weight > 1.0e-6f ? curvature / weight : fallback;
}

static void apaMocoResponseReference(const ApaMocoPathPointTransferV2Input_t *input,
                                     const ApaMocoPathProjection_t *projection,
                                     ApaMocoPathPointTransferV2Output_t *output) {
  output->ControlCurvature = output->RefCurvature;
  output->ControlCrossTrackError = output->CrossTrackError;
  output->ControlHeadingError = output->HeadingError;
  if (input->ResponseTime <= 0.0f || input->VehicleSpeed * input->ApaGoDir <= 0.0f)
    return;
  const Real distance =
      fminf(fabsf(input->VehicleSpeed) * input->ResponseTime, fmaxf(0.0f, projection->Remaining));
  const Real signedDistance = input->VehicleSpeed < 0.0f ? -distance : distance;
  const Real angle = signedDistance * tanf(input->SteeringAngle) / input->Wheelbase;
  Real lateralRatio = 0.0f;
  if (isfinite(input->VehicleLateralSpeed)) {
    const Real speed = hypotf(input->VehicleSpeed, input->VehicleLateralSpeed);
    if (speed > 0.15f && fabsf(input->VehicleSpeed) > 0.10f) {
      const Real weight = fminf(1.0f, fmaxf(0.0f, (speed - 0.15f) / 0.15f));
      const Real rawRatio = input->VehicleLateralSpeed / fabsf(input->VehicleSpeed);
      const Real deadbanded = copysignf(fmaxf(0.0f, fabsf(rawRatio) - 0.003f), rawRatio);
      lateralRatio = fminf(0.10f, fmaxf(-0.10f, weight * deadbanded));
    }
  }
  const Real lateralSlip = distance * lateralRatio;
  const Real a = fabsf(angle) > 1.e-5f ? sinf(angle) / angle : 1.0f;
  const Real b = fabsf(angle) > 1.e-5f ? (1.0f - cosf(angle)) / angle : 0.0f;
  const Real localX = a * signedDistance - b * lateralSlip;
  const Real localY = b * signedDistance + a * lateralSlip;
  const Real predictedX =
      input->VehicleX + cosf(input->VehicleYaw) * localX - sinf(input->VehicleYaw) * localY;
  const Real predictedY =
      input->VehicleY + sinf(input->VehicleYaw) * localX + cosf(input->VehicleYaw) * localY;
  const Real predictedYaw = input->VehicleYaw + angle;
  Real x, y, yaw;
  Integer index;
  apaMocoPathAtDistance(input->LocalPathX,
                        input->LocalPathY,
                        input->LocalPathYaw,
                        input->LocalPtCount,
                        projection->Progress + distance,
                        &index,
                        &x,
                        &y,
                        &yaw);
  output->ResponseDistance = distance;
  output->ControlCrossTrackError =
      -sinf(predictedYaw) * (x - predictedX) + cosf(predictedYaw) * (y - predictedY);
  output->ControlHeadingError = apaCoordWrapToHalfOpenPi(yaw - predictedYaw);
  output->ControlCurvature =
      apaMocoWeightedCurvature(input,
                               projection->Progress + 0.75f * distance,
                               fminf(projection->Length, projection->Progress + 1.25f * distance),
                               output->RefCurvature);
}

void apaMocoPathPointTransferV2(const ApaMocoPathPointTransferV2Input_t *input,
                                const ApaMocoPathPointTransferV2Param_t *param,
                                ApaMocoPathPointTransferV2State_t *state,
                                ApaMocoPathPointTransferV2Output_t *output) {
  ApaMocoPathProjection_t projection;
  Real preview, tx, ty, tyaw, dx, dy;
  Integer target, next, count = input->LocalPtCount;
  *output = (ApaMocoPathPointTransferV2Output_t){0};
  if (count < 2 || count > PATH_PT_MAX)
    return;
  if (!isfinite(input->VehicleSpeed) || !isfinite(param->PreviewTime) ||
      !isfinite(param->PreviewDistMin) || !isfinite(param->PreviewDistMax) ||
      param->PreviewTime < 0.0f || param->PreviewDistMin <= 0.0f ||
      param->PreviewDistMax < param->PreviewDistMin)
    return;
  if (!isfinite(input->ResponseTime) || input->ResponseTime < 0.0f ||
      (input->ResponseTime > 0.0f && (!isfinite(input->SteeringAngle) ||
                                      !isfinite(input->Wheelbase) || input->Wheelbase <= 0.0f)))
    return;
  if (state->LastSegmentStartIdx != input->SegmentStartIdx) {
    state->LastSegmentStartIdx = input->SegmentStartIdx;
    state->LastLocalIndex = 0;
    state->LastProgress = 0.0f;
  }
  if (input->UseProjection)
    projection = input->Projection;
  else
    apaMocoProjectPath(input->LocalPathX,
                       input->LocalPathY,
                       input->LocalPathYaw,
                       count,
                       input->VehicleX,
                       input->VehicleY,
                       input->VehicleYaw,
                       state->LastLocalIndex,
                       state->LastProgress,
                       param->SearchBackSteps,
                       param->SearchAheadSteps,
                       param->MaxProgressAdvance,
                       param->MaxReferenceDistance,
                       param->MaxHeadingError,
                       &projection);
  output->ReferenceValid = projection.Valid;
  output->Progress = projection.Progress;
  if (!projection.Valid)
    return;
  state->LastLocalIndex = projection.Index;
  state->LastProgress = fmaxf(state->LastProgress, projection.Progress);

  preview = param->PreviewDistMin + fabsf(input->VehicleSpeed) * param->PreviewTime;
  preview = fmaxf(param->PreviewDistMin, fminf(param->PreviewDistMax, preview));
  output->FeedbackDistance = preview;

  preview = fminf(preview, fmaxf(projection.Remaining, 0.0f));
  apaMocoPathAtDistance(input->LocalPathX,
                        input->LocalPathY,
                        input->LocalPathYaw,
                        count,
                        projection.Progress + preview,
                        &target,
                        &tx,
                        &ty,
                        &tyaw);
  dx = tx - input->VehicleX;
  dy = ty - input->VehicleY;
  output->RefX = output->TarPtX = tx;
  output->RefY = output->TarPtY = ty;
  output->RefYaw = output->TarPtYaw = tyaw;
  output->LocalRefIndex = output->CurLocalIndex = projection.Index;
  output->RefIndex =
      input->SegmentStartIdx + (Integer)((Real)(input->SegmentEndIdx - input->SegmentStartIdx) *
                                             (Real)projection.Index / (Real)(count - 1) +
                                         0.5f);
  output->TargetLocalIndex = target;
  output->LongitudinalError = cosf(input->VehicleYaw) * dx + sinf(input->VehicleYaw) * dy;
  output->LateralError = -sinf(input->VehicleYaw) * dx + cosf(input->VehicleYaw) * dy;
  output->CrossTrackError = projection.CrossTrackError;
  output->HeadingError = projection.HeadingError;
  output->GoalDistance = hypotf(input->LocalPathX[count - 1] - input->VehicleX,
                                input->LocalPathY[count - 1] - input->VehicleY);
  output->TerminalYawError =
      fabsf(apaCoordWrapToHalfOpenPi(input->LocalPathYaw[count - 1] - input->VehicleYaw));
  output->PathPointNum = count;
  output->PreviewDistance = input->ApaGoDir < 0 ? -preview : preview;
  output->PreviewClamped = preview < output->FeedbackDistance;
  next = target + 1;
  if (next >= count)
    next = count - 1;
  output->NxtPtX = input->LocalPathX[next];
  output->NxtPtY = input->LocalPathY[next];
  output->NxtPtYaw = input->LocalPathYaw[next];

  Integer current = projection.Index;
  if (current >= 0 && current + 1 < count) {
    Real ds = input->LocalPathOdo[current];
    if (fabsf(ds) > 1.0e-5f)
      output->RefCurvature = apaCoordWrapToHalfOpenPi(input->LocalPathYaw[current + 1] -
                                                      input->LocalPathYaw[current]) /
                             ds;
  }
  apaMocoResponseReference(input, &projection, output);
}
