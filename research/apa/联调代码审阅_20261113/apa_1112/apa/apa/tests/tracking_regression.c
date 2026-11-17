#include "apaMocoParkingControl.h"
#include "apaEnums.h"
#include "apaPathInterval.h"
#include "apaPathTypes.h"
#include "geoPthDiscretHtTypes.h"
#include "vehicleDynamicStepTypes.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>

static ApaMocoParkingControlInput_t reverse_then_forward(void) {
  ApaMocoParkingControlInput_t in = {0};
  int i;
  in.Chart.PtCount = 17;
  in.Chart.PathUpdateIdx = 1;
  in.Chart.TargetSlotKind = 1;
  in.Chart.ParkingAction = 1;
  in.Chart.GearPos = APA_GEAR_POS_R;
  in.Chart.GoalX = 4.0f;
  for (i = 0; i < 17; ++i) {
    in.Chart.PthPtX[i] = i <= 6 ? -(float)i : (float)i - 12.0f;
    in.Chart.PthPtOdo[i] = i < 6 ? -1.0f : i < 16 ? 1.0f : 0.0f;
  }
  return in;
}

static void no_early_switch(void) {
  ApaMocoParkingControlInput_t in = reverse_then_forward();
  ApaMocoParkingControlParam_t p;
  ApaMocoParkingControlState_t s = {0};
  ApaMocoParkingControlOutput_t out;
  int i;
  apaMocoParkingControlDefaultParam(&p);
  for (i = 0; i < 100; ++i) {
    in.VehicleSpeed = i == 38 ? -0.488f : 0.0f;
    in.VehicleAcceleration = i == 38 ? -2.641f : 0.0f;
    apaMocoParkingControl(&in, &p, &s, &out);
    assert(out.Chart.ApaStpNow == 1);
    assert(out.Chart.TargetGearReq == APA_GEAR_POS_R);
    assert(out.Chart.CtrlPhase != APA_MOCO_DONE);
    if (i == 38) {
      printf("surge braking distance=%.5f remaining=%.5f\n", out.StopDistance, out.Chart.Dist2Go);
      assert(out.StopDistance < 3.0f);
    }
  }
  for (; i < 700; ++i) apaMocoParkingControl(&in, &p, &s, &out);
  assert(out.Chart.CtrlPhase == APA_MOCO_FAULT);
  assert(!out.Longi.ControlEnabled && out.Longi.StopReq);
  assert(out.Chart.ApaStpNow == 1);
}

static void stopped_short_and_cusp_hold(void) {
  ApaMocoParkingControlInput_t in = reverse_then_forward();
  ApaMocoParkingControlParam_t p;
  ApaMocoParkingControlState_t s = {0};
  ApaMocoParkingControlOutput_t out;
  int i;
  apaMocoParkingControlDefaultParam(&p);
  for (i = 0; i < 12; ++i) apaMocoParkingControl(&in, &p, &s, &out);
  for (i = 1; i <= 118; ++i) {
    in.Chart.VehicleX = -0.05f * i;
    in.VehicleSpeed = -0.1f;
    apaMocoParkingControl(&in, &p, &s, &out);
    assert(out.Chart.ApaStpNow == 1);
  }
  in.VehicleSpeed = 0.0f;
  for (i = 0; i < 30; ++i) {
    apaMocoParkingControl(&in, &p, &s, &out);
    assert(out.Chart.ApaStpNow == 1);
    assert(out.Chart.CtrlPhase == APA_MOCO_CREEP);
  }
  in.Chart.VehicleX = -5.98f;

  in.Chart.VehicleY = 0.05f;
  in.Chart.VehicleYaw = 0.02f;
  in.VehicleSpeed = -0.10f;
  apaMocoParkingControl(&in, &p, &s, &out);
  assert(out.Chart.CtrlPhase == APA_MOCO_STOP_WAIT);
  assert(out.Longi.ControlEnabled && !out.Longi.StopReq);
  for (i = 0; i < 20; ++i) {
    apaMocoParkingControl(&in, &p, &s, &out);
    assert(out.Chart.ApaStpNow == 1);
  }
  in.VehicleSpeed = 0.0f;
  for (i = 0; i < p.Chart.StopDwellCycles; ++i) apaMocoParkingControl(&in, &p, &s, &out);
  assert(out.Chart.ApaStpNow == 1);
  apaMocoParkingControl(&in, &p, &s, &out);
  assert(out.Chart.ApaStpNow == 2);
  assert(out.Chart.TargetGearReq == APA_GEAR_POS_D);
  assert(!out.Longi.ControlEnabled);
}

static void wrong_heading_and_lost_reference(void) {
  ApaMocoParkingControlInput_t in = reverse_then_forward();
  ApaMocoParkingControlParam_t p;
  ApaMocoParkingControlState_t s = {0};
  ApaMocoParkingControlOutput_t out;
  apaMocoParkingControlDefaultParam(&p);
  apaMocoParkingControl(&in, &p, &s, &out);
  in.Chart.VehicleYaw = 3.14159265f;
  in.Chart.VehicleX = 3.99f;
  apaMocoParkingControl(&in, &p, &s, &out);
  assert(out.Chart.CtrlPhase == APA_MOCO_FAULT && !out.Chart.DoneReady);
  assert(out.Longi.StopReq && !out.Longi.ControlEnabled);
  in.Chart.VehicleX = 0.0f;
  in.Chart.VehicleYaw = 0.0f;
  apaMocoParkingControl(&in, &p, &s, &out);
  assert(out.Chart.CtrlPhase == APA_MOCO_FAULT);
  in.Chart.PathUpdateIdx++;
  apaMocoParkingControl(&in, &p, &s, &out);
  assert(out.Chart.CtrlPhase == APA_MOCO_GEAR_CHECK);
  in.VehicleAcceleration = NAN;
  apaMocoParkingControl(&in, &p, &s, &out);
  assert(out.Longi.StopReq && !out.Longi.ControlEnabled);
  in.VehicleAcceleration = 0.0f;
  apaMocoParkingControl(&in, &p, &s, &out);
  assert(out.Chart.CtrlPhase == APA_MOCO_FAULT);
}

static void closed_loop_straight(void) {
  ApaMocoParkingControlInput_t in = reverse_then_forward();
  ApaMocoParkingControlParam_t p;
  ApaMocoParkingControlState_t s = {0};
  ApaMocoParkingControlOutput_t out;
  VehicleDynamicStepParam_t plant = {0};
  VehicleDynamicStepState_t ps = {0};
  VehicleDynamicStepInput_t pi = {0};
  VehicleDynamicStepOutput_t po;
  int i, last = 1;
  apaMocoParkingControlDefaultParam(&p);
  p.Longi.ForwardRefSpeed = 0.8f;
  p.Longi.ReverseRefSpeed = -0.65f;
  plant.CycleTime = p.Longi.CycleTime;
  plant.WheelBase = p.Lat.Wheelbase;
  plant.ForwardSpeedLimit = p.Longi.SpeedUpperLimit;
  plant.ReverseSpeedLimit = -p.Longi.SpeedLowerLimit;
  plant.AccelerationUpperLimit = p.Longi.AccelerationUpperLimit;
  plant.AccelerationLowerLimit = p.Longi.AccelerationLowerLimit;
  plant.StraightSteerThreshold = 0.0001f;
  for (i = 0; i < 6500; ++i) {
    apaMocoParkingControl(&in, &p, &s, &out);
    if (out.Chart.CtrlPhase == APA_MOCO_FAULT) {
      printf("fault cycle=%d segment=%d reason=%d x=%.6f remaining=%.6f speed=%.6f\n", i,
          out.Chart.ApaStpNow, out.Chart.StopReason, in.Chart.VehicleX, out.Chart.Dist2Go, in.VehicleSpeed);
      assert(out.Chart.CtrlPhase != APA_MOCO_FAULT);
    }
    if (out.Chart.ApaStpNow != last) {
      assert(fabsf(in.Chart.VehicleX + 6.0f) <= p.Chart.CuspPositionTolerance);
      assert(fabsf(in.VehicleSpeed) <= p.Chart.StandstillSpeed);
      last = out.Chart.ApaStpNow;
    }
    if (out.Chart.CtrlPhase == APA_MOCO_DONE) break;
    pi.VehicleX = in.Chart.VehicleX;
    pi.VehicleY = in.Chart.VehicleY;
    pi.VehicleYaw = in.Chart.VehicleYaw;
    pi.GearPos = in.Chart.GearPos;
    if (out.Longi.GearChangeAllowed) pi.GearPos = out.Chart.TargetGearReq;
    pi.AccelerationCmd = out.Longi.TargetAcceleration;
    if (out.Longi.StopReq) pi.AccelerationCmd = in.VehicleSpeed > 0 ? -p.Longi.StopDeceleration :
        in.VehicleSpeed < 0 ? p.Longi.StopDeceleration : 0.0f;
    pi.SteerAngle = out.Lat.SteerAngle;
    vehicleDynamicStep(&pi, &plant, &ps, &po);
    in.Chart.VehicleX = po.VehicleX;
    in.Chart.VehicleY = po.VehicleY;
    in.Chart.VehicleYaw = po.VehicleYaw;
    in.Chart.GearPos = pi.GearPos;
    in.VehicleSpeed = po.VehicleSpeed;
    in.VehicleAcceleration = po.VehicleAcceleration;
  }
  printf("straight closed loop cycles=%d x=%.6f speed=%.6f\n", i, in.Chart.VehicleX, in.VehicleSpeed);
  assert(i < 6500);
  assert(fabsf(in.Chart.VehicleX - 4.0f) <= p.Chart.GoalPositionTolerance);
  assert(fabsf(in.VehicleSpeed) <= p.Chart.StandstillSpeed);
}

static void distance_profile_owns_approach(void) {
  ApaMocoCtrlChartV2Input_t in = reverse_then_forward().Chart;
  ApaMocoParkingControlParam_t p;
  ApaMocoCtrlChartV2State_t s = {0};
  ApaMocoCtrlChartV2Output_t out;
  int i;
  apaMocoParkingControlDefaultParam(&p);
  in.ApproachDistance = p.Longi.SlowDistThd;
  in.CreepDistance = p.Longi.TerminalDistThd;
  in.BrakeDistance = 10.0f;
  for (i = 0; i < 30; ++i) apaMocoCtrlChartV2(&in, &p.Chart, &s, &out);

  assert(out.CtrlPhase == APA_MOCO_TRACK);
  in.VehicleSpeed = -0.1f;
  in.BrakeDistance = 0.01f;
  for (i = 1; i <= 54; ++i) {
    in.VehicleX = -0.1f * (float)i;
    apaMocoCtrlChartV2(&in, &p.Chart, &s, &out);
  }
  assert(out.CtrlPhase == APA_MOCO_DECEL);
  assert(out.Dist2Go > 0.4f && out.Dist2Go < 0.8f);
}

static void approach_demand_and_arrival_braking(void) {
  ApaMocoParkingControlParam_t p;
  ApaMocoLongiCtrlV2Input_t in = {0};
  ApaMocoLongiCtrlV2State_t s = {0};
  ApaMocoLongiCtrlV2Output_t out;
  int i;
  apaMocoParkingControlDefaultParam(&p);
  in.CtrlPhase = APA_MOCO_DECEL;
  in.ApaStpNow = 1;
  in.ApaStpNum = 2;
  in.ApaGoDir = -1;
  in.SegmentLength = 1.0f;
  in.Dist2Go = 0.3f;
  in.VehicleSpeed = -0.4f;
  in.TargetGearReq = in.GearPos = APA_GEAR_POS_R;
  for (i = 0; i < 60; ++i) apaMocoLongiCtrlV2(&in, &p.Longi, &s, &out);
  assert(out.ControlEnabled && !out.StopReq);
  assert(out.TargetSpeed < 0.0f && out.TargetSpeed > in.VehicleSpeed);
  assert(out.TargetAcceleration > 0.0f && out.TargetAcceleration <= p.Longi.StopDeceleration);
  in.CtrlPhase = APA_MOCO_STOP_WAIT;
  for (i = 0; i < 60; ++i) apaMocoLongiCtrlV2(&in, &p.Longi, &s, &out);
  assert(out.ControlEnabled && !out.StopReq);
  assert(out.TargetAcceleration >= 1.1999f);
}

static void wrapped_heading_continuity_is_nonnegative(void) {
  ApaPath_t path = {0};
  const float degree = APA_PI / 180.0f;
  path.PathN = 3;
  path.Tht[0] = 179.0f * degree;
  path.Tht[1] = -179.0f * degree;
  path.Tht[2] = -177.0f * degree;
  assert(fabsf(apaPathEvaluateContinuity(&path) - 4.0f * degree) < 1.e-5f);
  path.Tht[0] = -179.0f * degree;
  path.Tht[1] = 179.0f * degree;
  path.Tht[2] = 177.0f * degree;
  assert(fabsf(apaPathEvaluateContinuity(&path) - 4.0f * degree) < 1.e-5f);
}

static void curve_speed_is_an_absolute_constraint(void) {
  ApaMocoParkingControlInput_t in = {0};
  ApaMocoParkingControlParam_t p;
  ApaMocoParkingControlState_t s = {0};
  ApaMocoParkingControlOutput_t out;
  apaMocoParkingControlDefaultParam(&p);
  in.Chart.PtCount = 41;
  in.Chart.PathUpdateIdx = 1;
  in.Chart.TargetSlotKind = 1;
  in.Chart.ParkingAction = 1;
  in.Chart.GearPos = APA_GEAR_POS_R;
  const Real curvature = tanf(p.Lat.SteerUpperLimit)/p.Lat.Wheelbase;
  for (int i=0; i<41; ++i) {
    const Real distance = -0.1f*i;
    in.Chart.PthPtX[i] = sinf(curvature*distance)/curvature;
    in.Chart.PthPtY[i] = (1.0f-cosf(curvature*distance))/curvature;
    in.Chart.PthPtTht[i] = curvature*distance;
    in.Chart.PthPtOdo[i] = i<40 ? -0.1f : 0.0f;
  }
  in.Chart.GoalX = in.Chart.PthPtX[40];
  in.Chart.GoalY = in.Chart.PthPtY[40];
  in.Chart.GoalYaw = in.Chart.PthPtTht[40];
  for (int i=0; i<20; ++i) apaMocoParkingControl(&in,&p,&s,&out);
  assert(fabsf(out.CurveSpeedLimit-p.CurveRefSpeed)<0.002f);
  assert(fabsf(out.Longi.TargetSpeed-p.Longi.ReverseRefSpeed)>0.1f);
  assert(fabsf(out.Longi.TargetSpeed)<=p.CurveRefSpeed+0.002f);
  p.Longi.ReverseRefSpeed = -0.8f;
  apaMocoParkingControl(&in,&p,&s,&out);
  assert(fabsf(out.Longi.TargetSpeed+p.CurveRefSpeed)<0.002f);
  in.AvailableDistanceLimited = true;
  in.AvailableDistance = 0.10f;
  apaMocoParkingControl(&in,&p,&s,&out);
  assert(out.ObstacleSpeedLimit<out.CurveSpeedLimit);
  assert(fabsf(out.Longi.TargetSpeed)<=out.ObstacleSpeedLimit+1.e-5f);
}

static void response_preview_stays_in_active_segment(void) {
  ApaMocoPathPointTransferV2Input_t in = {0};
  ApaMocoParkingControlParam_t p;
  ApaMocoPathPointTransferV2State_t state = {0};
  ApaMocoPathPointTransferV2Output_t out;
  apaMocoParkingControlDefaultParam(&p);
  in.LocalPtCount = 3;
  in.LocalPathX[1] = -0.10f;
  in.LocalPathX[2] = -0.20f;
  in.LocalPathOdo[0] = in.LocalPathOdo[1] = -0.10f;
  in.VehicleX = -0.19f;
  in.VehicleSpeed = -0.6f;
  in.ApaGoDir = -1;
  in.Wheelbase = p.Lat.Wheelbase;
  in.ResponseTime = p.SteeringResponseTime;
  apaMocoPathPointTransferV2(&in,&p.Path,&state,&out);
  assert(out.ReferenceValid);
  assert(fabsf(out.ResponseDistance-0.01f)<1.e-4f);
  assert(fabsf(out.ControlCurvature)<1.e-5f);
  assert(fabsf(out.ControlCrossTrackError)<1.e-5f);
  assert(fabsf(out.ControlHeadingError)<1.e-5f);
}

static void scurve_reversal_lowers_curve_speed_limit(void) {
  ApaMocoParkingControlInput_t in = {0};
  ApaMocoParkingControlParam_t p;
  ApaMocoParkingControlState_t s = {0};
  ApaMocoParkingControlOutput_t out;
  apaMocoParkingControlDefaultParam(&p);
  in.Chart.PtCount = 41;
  in.Chart.PathUpdateIdx = 1;
  in.Chart.TargetSlotKind = 2;
  in.Chart.ParkingAction = 1;
  in.Chart.GearPos = APA_GEAR_POS_R;
  const Real curvature = tanf(p.Lat.SteerUpperLimit) / p.Lat.Wheelbase;
  Real x = 0.0f, y = 0.0f, yaw = 0.0f;
  for (int i = 0; i < 41; ++i) {
    in.Chart.PthPtX[i] = x;
    in.Chart.PthPtY[i] = y;
    in.Chart.PthPtTht[i] = yaw;
    in.Chart.PthPtOdo[i] = i < 40 ? -0.1f : 0.0f;
    const Real k = i < 20 ? curvature : -curvature;
    const Real ds = -0.1f;
    const Real dyaw = k * ds;
    x += cosf(yaw + 0.5f * dyaw) * ds;
    y += sinf(yaw + 0.5f * dyaw) * ds;
    yaw += dyaw;
  }
  in.Chart.GoalX = in.Chart.PthPtX[40];
  in.Chart.GoalY = in.Chart.PthPtY[40];
  in.Chart.GoalYaw = in.Chart.PthPtTht[40];
  for (int i = 0; i < 20; ++i) apaMocoParkingControl(&in, &p, &s, &out);
  const Real singleArcSpeed = out.CurveSpeedLimit;
  assert(fabsf(singleArcSpeed - p.CurveRefSpeed) < 0.002f);
  in.VehicleSpeed = -0.35f;
  for (int i = 1; i <= 16; ++i) {
    in.Chart.VehicleX = in.Chart.PthPtX[i];
    in.Chart.VehicleY = in.Chart.PthPtY[i];
    in.Chart.VehicleYaw = in.Chart.PthPtTht[i];
    apaMocoParkingControl(&in, &p, &s, &out);
  }
  assert(out.CurveSpeedLimit < 0.25f);
}

static void lateral_slip_damping_opposes_drift_and_preserves_zero_slip(void) {
  ApaMocoParkingControlParam_t p;
  apaMocoParkingControlDefaultParam(&p);
  for (int direction = -1; direction <= 1; direction += 2) {
    ApaMocoLatCtrlV2Input_t in = {0};
    ApaMocoLatCtrlV2State_t s0 = {0}, sLeft = {0}, sRight = {0};
    ApaMocoLatCtrlV2Output_t out0, outLeft, outRight;
    in.CtrlPhase = APA_MOCO_TRACK;
    in.ApaGoDir = direction;
    in.PathPointNum = 20;
    in.Dist2Go = 2.0f;
    in.FeedbackDistance = 0.7f;
    in.TargetSpeed = direction * 0.40f;
    in.VehicleSpeed = direction * 0.40f;
    in.CycleTime = 0.02f;
    apaMocoLatCtrlV2(&in, &p.Lat, &s0, &out0);
    assert(fabsf(out0.SteerAngle) < 1.e-6f);

    in.VehicleLateralSpeed = 0.03f;
    apaMocoLatCtrlV2(&in, &p.Lat, &sLeft, &outLeft);
    assert(outLeft.SteerAngle < -0.01f);

    in.VehicleLateralSpeed = -0.03f;
    apaMocoLatCtrlV2(&in, &p.Lat, &sRight, &outRight);
    assert(outRight.SteerAngle > 0.01f);

    in.VehicleSpeed = direction * 0.10f;
    sLeft = (ApaMocoLatCtrlV2State_t){0};
    apaMocoLatCtrlV2(&in, &p.Lat, &sLeft, &outLeft);
    assert(fabsf(outLeft.SteerAngle) < 1.e-6f);
  }
}

static void parallel_scurve_discretization_keeps_intervals_consistent(void) {

  const Real lengths[4] = {0.49f, -0.49f, 0.486f, -0.25f};
  const Real curvatures[4] = {0.25f, -0.25f, 0.25f, -0.25f};
  GeoPthDiscretHtInput_t in = {0};
  GeoPthDiscretHtOutput_t out = {0};
  Real x = 0.0f, y = 0.0f, th = 0.0f;
  int i;
  in.SegNum = 4;
  for (i = 0; i < 4; ++i) {
    const Real d = lengths[i];
    const Real k = curvatures[i];
    const Real dth = d * k;
    const Real cx = x - sinf(th) / k;
    const Real cy = y + cosf(th) / k;
    in.SegGoDir[i] = d >= 0.0f ? 1 : -1;
    in.SegTrnDir[i] = k >= 0.0f ? 1 : -1;
    in.SegCrv[i] = k;
    in.SegTht[i] = dth;
    in.SegLen[i] = fabsf(d);
    in.SegStX[i] = x;
    in.SegStY[i] = y;
    in.SegStTh[i] = th;
    in.SegCcX[i] = cx;
    in.SegCcY[i] = cy;
    in.PathLen += fabsf(d);
    th += dth;
    x = cx + sinf(th) / k;
    y = cy - cosf(th) / k;
  }
  in.TarPtX = x;
  in.TarPtY = y;
  in.TarPtTheta = th;
  geoPthDiscretHtPkIn(&in, &out);
  assert(out.PtCount > 10);
  assert(hypotf(out.PthPtX[out.PtCount - 1] - x,
                out.PthPtY[out.PtCount - 1] - y) < 1.e-4f);
  for (i = 0; i + 1 < out.PtCount; ++i) {
    assert(apaPathIntervalIsConsistent(
        out.PthPtX[i], out.PthPtY[i], out.PthPtTht[i], out.PthPtOdo[i],
        out.PthPtX[i + 1], out.PthPtY[i + 1], out.PthPtTht[i + 1]));
  }
}

int main(void) {
  curve_speed_is_an_absolute_constraint();
  scurve_reversal_lowers_curve_speed_limit();
  lateral_slip_damping_opposes_drift_and_preserves_zero_slip();
  parallel_scurve_discretization_keeps_intervals_consistent();
  response_preview_stays_in_active_segment();
  wrapped_heading_continuity_is_nonnegative();
  no_early_switch();
  stopped_short_and_cusp_hold();
  wrong_heading_and_lost_reference();
  closed_loop_straight();
  distance_profile_owns_approach();
  approach_demand_and_arrival_braking();
  puts("APA tracking regression passed");
  return 0;
}
