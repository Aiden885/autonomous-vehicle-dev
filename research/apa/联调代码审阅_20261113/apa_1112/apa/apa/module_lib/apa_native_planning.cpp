#include "apa_native_planning.h"

#include <atomic>
#include <limits>
#include <stdexcept>
#include <type_traits>
#include <unordered_map>

extern "C" {
#include "apaVtParkingPathPlanning.h"
#include "apaHtParkingPathPlanning.h"
#include "apaDthEdVtParkingPathPlanning.h"
#include "apaVhParkingPathPlanning.h"
#include "apaParkingOutPathPlanning.h"
#include "apaParkingOutTargetConnector.h"
#include "apaParkingPathSafetyGate.h"
#include "apaTrackingPolicy.h"
}

namespace apa::native {
namespace {
std::atomic<UInteger> next_scene_id{1};
thread_local std::unordered_map<UInteger, ApaPlanningSceneQuery_t> scenes;

ApaPlanningSceneQuery_t Query(UInteger id) {
  if (id == 0) return {};
  const auto found = scenes.find(id);
  if (found == scenes.end()) throw std::runtime_error("APA scene resource expired or belongs to another thread");
  return found->second;
}

ApaParkingPathPlanningInput_t NativeInput(const ApaParkingPathPlanningInputData& input) {
  ApaParkingPathPlanningInput_t value{};
  value.ParkingAction = input.parkingAction;
  value.TargetSlotKind = input.targetSlotKind;
  value.VehicleX = input.vehicleX;
  value.VehicleY = input.vehicleY;
  value.VehicleYaw = input.vehicleYaw;
  value.TarX = input.targetX;
  value.TarY = input.targetY;
  value.TarTheta = input.targetTheta;
  value.GoalPositionTolerance = input.goalPositionTolerance;
  value.GoalYawTolerance = input.goalYawTolerance;
  value.Rmin = input.minimumTurningRadius;
  value.EntrPtX = input.entrancePointX;
  value.EntrPtY = input.entrancePointY;
  value.SltPt1X = input.slotPoint1X;
  value.SltPt1Y = input.slotPoint1Y;
  value.SltPt2X = input.slotPoint2X;
  value.SltPt2Y = input.slotPoint2Y;
  value.SltPt3X = input.slotPoint3X;
  value.SltPt3Y = input.slotPoint3Y;
  value.SltPt4X = input.slotPoint4X;
  value.SltPt4Y = input.slotPoint4Y;
  value.SltEntr = input.slotEntrance;
  value.PrkType = input.parkingType;
  value.SurdType = input.surroundingType;
  value.AgrOfst = input.availableSpaceOffset;
  value.BoundaryLineNum = input.boundaryLineCount;
  value.VirtualBoundaryLineNum = input.virtualBoundaryLineCount;
  value.SlotTrackStopperNum = input.slotTrackStopperCount;
  value.BoundarySepPtNum = input.boundarySepPointCount;
  value.StopperPtNum = input.stopperPointCount;
  value.ShortCutSw = input.shortcutSwitch;
  value.ShortCutThd = input.shortcutThreshold;
  for (Integer i = 0; i < 64; ++i) {
    value.BoundaryLineX1[i] = input.boundaryLineX1[i];
    value.BoundaryLineY1[i] = input.boundaryLineY1[i];
    value.BoundaryLineX2[i] = input.boundaryLineX2[i];
    value.BoundaryLineY2[i] = input.boundaryLineY2[i];
    value.VirtualBoundaryP1X[i] = input.virtualBoundaryPoint1X[i];
    value.VirtualBoundaryP1Y[i] = input.virtualBoundaryPoint1Y[i];
    value.VirtualBoundaryP2X[i] = input.virtualBoundaryPoint2X[i];
    value.VirtualBoundaryP2Y[i] = input.virtualBoundaryPoint2Y[i];
    value.SlotTrackStopperX1[i] = input.slotTrackStopperX1[i];
    value.SlotTrackStopperY1[i] = input.slotTrackStopperY1[i];
    value.SlotTrackStopperX2[i] = input.slotTrackStopperX2[i];
    value.SlotTrackStopperY2[i] = input.slotTrackStopperY2[i];
  }
  for (Integer i = 0; i < 128; ++i) {
    value.BoundarySepPtX[i] = input.boundarySepPointX[i];
    value.BoundarySepPtY[i] = input.boundarySepPointY[i];
    value.StopperPtX[i] = input.stopperPointX[i];
    value.StopperPtY[i] = input.stopperPointY[i];
  }
  return value;
}

ApaParkingPathPlanningInputData ModelInput(const ApaParkingPathPlanningInput_t& input) {
  ApaParkingPathPlanningInputData value{};
  value.parkingAction = input.ParkingAction;
  value.targetSlotKind = input.TargetSlotKind;
  value.vehicleX = input.VehicleX;
  value.vehicleY = input.VehicleY;
  value.vehicleYaw = input.VehicleYaw;
  value.targetX = input.TarX;
  value.targetY = input.TarY;
  value.targetTheta = input.TarTheta;
  value.goalPositionTolerance = input.GoalPositionTolerance;
  value.goalYawTolerance = input.GoalYawTolerance;
  value.minimumTurningRadius = input.Rmin;
  value.entrancePointX = input.EntrPtX;
  value.entrancePointY = input.EntrPtY;
  value.slotPoint1X = input.SltPt1X;
  value.slotPoint1Y = input.SltPt1Y;
  value.slotPoint2X = input.SltPt2X;
  value.slotPoint2Y = input.SltPt2Y;
  value.slotPoint3X = input.SltPt3X;
  value.slotPoint3Y = input.SltPt3Y;
  value.slotPoint4X = input.SltPt4X;
  value.slotPoint4Y = input.SltPt4Y;
  value.slotEntrance = input.SltEntr;
  value.parkingType = input.PrkType;
  value.surroundingType = input.SurdType;
  value.availableSpaceOffset = input.AgrOfst;
  value.boundaryLineCount = input.BoundaryLineNum;
  value.virtualBoundaryLineCount = input.VirtualBoundaryLineNum;
  value.slotTrackStopperCount = input.SlotTrackStopperNum;
  value.boundarySepPointCount = input.BoundarySepPtNum;
  value.stopperPointCount = input.StopperPtNum;
  value.shortcutSwitch = input.ShortCutSw;
  value.shortcutThreshold = input.ShortCutThd;
  for (Integer i = 0; i < 64; ++i) {
    value.boundaryLineX1[i] = input.BoundaryLineX1[i];
    value.boundaryLineY1[i] = input.BoundaryLineY1[i];
    value.boundaryLineX2[i] = input.BoundaryLineX2[i];
    value.boundaryLineY2[i] = input.BoundaryLineY2[i];
    value.virtualBoundaryPoint1X[i] = input.VirtualBoundaryP1X[i];
    value.virtualBoundaryPoint1Y[i] = input.VirtualBoundaryP1Y[i];
    value.virtualBoundaryPoint2X[i] = input.VirtualBoundaryP2X[i];
    value.virtualBoundaryPoint2Y[i] = input.VirtualBoundaryP2Y[i];
    value.slotTrackStopperX1[i] = input.SlotTrackStopperX1[i];
    value.slotTrackStopperY1[i] = input.SlotTrackStopperY1[i];
    value.slotTrackStopperX2[i] = input.SlotTrackStopperX2[i];
    value.slotTrackStopperY2[i] = input.SlotTrackStopperY2[i];
  }
  for (Integer i = 0; i < 128; ++i) {
    value.boundarySepPointX[i] = input.BoundarySepPtX[i];
    value.boundarySepPointY[i] = input.BoundarySepPtY[i];
    value.stopperPointX[i] = input.StopperPtX[i];
    value.stopperPointY[i] = input.StopperPtY[i];
  }
  return value;
}

ApaParkingPathPlanningParam_t NativeParam(const ApaParkingPathPlanningInputData& input,
    const ApaPlanningCalibration& calibration) {
  ApaParkingPathPlanningParam_t value{};
  value.SltState = input.slotState;
  value.VehW = calibration.vehicleWidth;
  value.VehLlF = calibration.vehicleFrontEnvelopeLength;
  value.VehLfMin = calibration.minimumFrontSafetyLength;
  value.VehLr = calibration.vehicleRearOverhang;
  value.VtFallbackSpace = calibration.verticalFallbackSpace;
  value.DthFallbackSpace = calibration.diagonalFallbackSpace;
  value.HtRfoutMin = calibration.parallelMinimumExitRadius;
  value.NeedHybridAlgFusion = input.needHybridAlgorithmFusion;
  value.SceneQuery = Query(input.sceneQueryId);
  value.SceneQuery.TrackingReserve = APA_PLANNING_TRACKING_RESERVE_M;
  return value;
}

void ModelOutput(const ApaParkingPathPlanningOutput_t& input, ApaParkingPathPlanningOutputData& output) {
  output.selectedSlotKind = input.SelectedSlotKind;
  output.planFlag = input.PlanFlg;
  output.errorType = input.ErrorType;
  output.plannerCase = input.PlannerCase;
  output.segmentCount = input.SegNum;
  output.directionChangeCount = input.StpN;
  output.pathLength = input.PathLen;
  output.pathPointCount = input.PtCount;
  output.maneuverStepCount = input.StepCount;
  output.secondStageEndPointOrdinal = input.St2Idx;
  output.firstForwardPointVehicleX = input.Stp1PtRX;
  output.firstForwardPointVehicleY = input.Stp1PtRY;
  output.firstForwardPointVehicleTheta = input.Stp1PtRTheta;
  output.planOk = input.PlanFlg != 0;
  for (Integer i = 0; i < 100; ++i) {
    output.pathPointX[i] = input.PthPtX[i];
    output.pathPointY[i] = input.PthPtY[i];
    output.pathPointTheta[i] = input.PthPtTht[i];
    output.pathPointOdometer[i] = input.PthPtOdo[i];
  }
}

ApaParkingPathPlanningOutput_t NativeOutput(const ApaParkingPathPlanningOutputData& input) {
  ApaParkingPathPlanningOutput_t output{};
  output.SelectedSlotKind = input.selectedSlotKind;
  output.PlanFlg = input.planFlag;
  output.ErrorType = input.errorType;
  output.PlannerCase = input.plannerCase;
  output.SegNum = input.segmentCount;
  output.StpN = input.directionChangeCount;
  output.PathLen = input.pathLength;
  output.PtCount = input.pathPointCount;
  output.StepCount = input.maneuverStepCount;
  output.St2Idx = input.secondStageEndPointOrdinal;
  output.Stp1PtRX = input.firstForwardPointVehicleX;
  output.Stp1PtRY = input.firstForwardPointVehicleY;
  output.Stp1PtRTheta = input.firstForwardPointVehicleTheta;
  for (Integer i = 0; i < 100; ++i) {
    output.PthPtX[i] = input.pathPointX[i];
    output.PthPtY[i] = input.pathPointY[i];
    output.PthPtTht[i] = input.pathPointTheta[i];
    output.PthPtOdo[i] = input.pathPointOdometer[i];
  }
  return output;
}
}

SceneScope::SceneScope(ApaPlanningSceneQuery_t query) {
  if (!query.CheckPose) throw std::invalid_argument("APA full scene requires a collision callback");
  auto candidate = next_scene_id.load();
  do {
    if (candidate == std::numeric_limits<UInteger>::max())
      throw std::overflow_error("APA scene resource IDs exhausted");
  } while (!next_scene_id.compare_exchange_weak(candidate, candidate + 1));
  id_ = candidate;
  scenes.emplace(id_, query);
}

SceneScope::~SceneScope() { scenes.erase(id_); }

ApaParkingManeuverAssessment_t AssessPath(const ApaParkingPathPlanningInputData& input,
    const ApaParkingPathPlanningOutputData& path, Real reserve) {
  const auto request = NativeInput(input);
  const auto param = NativeParam(input, global::params.planning);
  const auto candidate = NativeOutput(path);
  return apaParkingAssessManeuver(&candidate, &request, &param, reserve);
}

void PlanSlot(const ApaParkingPathPlanningInputData& input,
    const ApaPlanningCalibration& calibration, ApaParkingPathPlanningOutputData& output,
    ApaParkingPathPlanningInputData& resolvedInput) {
  output = {};
  resolvedInput = input;
  ApaParkingPathPlanningFromSlotInput_t request{};
  request.PlannerInput = NativeInput(input);
  request.AngleUnit = APA_SLOT_REQUEST_ANGLE_RAD;
  ApaParkingPathPlanningFromSlotParam_t param{};
  apaParkingPathPlanningFromSlotDefaultParam(&param);
  param.PlannerParam = NativeParam(input, calibration);
  ApaParkingPathPlanningFromSlotOutput_t result{};
  apaParkingPathPlanningFromSlot(&request, &param, &result);
  ModelOutput(result.Path, output);
  output.resolvedScene = ModelInput(result.ResolvedInput);

  output.resolvedScene.sceneQueryId = 0;
  output.resolvedScene.slotState = input.slotState;
  output.resolvedScene.needHybridAlgorithmFusion = input.needHybridAlgorithmFusion;
  if (result.Path.PlanFlg != 0) {
    resolvedInput.vehicleX = result.ResolvedInput.VehicleX;
    resolvedInput.vehicleY = result.ResolvedInput.VehicleY;
    resolvedInput.vehicleYaw = result.ResolvedInput.VehicleYaw;
    resolvedInput.targetX = result.ResolvedInput.TarX;
    resolvedInput.targetY = result.ResolvedInput.TarY;
    resolvedInput.targetTheta = result.ResolvedInput.TarTheta;
    resolvedInput.entrancePointX = result.ResolvedInput.EntrPtX;
    resolvedInput.entrancePointY = result.ResolvedInput.EntrPtY;
    resolvedInput.parkingType = result.ResolvedInput.PrkType;
  }
}

void PlanVt(const ApaParkingPathPlanningInputData& input,
    const ApaPlanningCalibration& calibration, ApaParkingPathPlanningOutputData& output) {
  const auto request = NativeInput(input);
  const auto param = NativeParam(input, calibration);
  ApaVtParkingPathPlanningOutput_t result{};
  ::apaVtParkingPathPlanning(&request, &param, &result);
  ModelOutput(result, output);
}

void PlanHt(const ApaParkingPathPlanningInputData& input,
    const ApaPlanningCalibration& calibration, ApaParkingPathPlanningOutputData& output) {
  const auto request = NativeInput(input);
  const auto param = NativeParam(input, calibration);
  ApaHtParkingPathPlanningOutput_t result{};
  ::apaHtParkingPathPlanning(&request, &param, &result);
  ModelOutput(result, output);
}

void PlanDthEdVt(const ApaParkingPathPlanningInputData& input,
    const ApaPlanningCalibration& calibration, ApaParkingPathPlanningOutputData& output) {
  const auto request = NativeInput(input);
  const auto param = NativeParam(input, calibration);
  ApaDthEdVtParkingPathPlanningOutput_t result{};
  ::apaDthEdVtParkingPathPlanning(&request, &param, &result);
  ModelOutput(result, output);
}

void PlanVh(const ApaParkingPathPlanningInputData& input,
    const ApaPlanningCalibration& calibration, ApaParkingPathPlanningOutputData& output) {
  const auto request = NativeInput(input);
  const auto param = NativeParam(input, calibration);
  ApaVhParkingPathPlanningOutput_t result{};
  ::apaVhParkingPathPlanning(&request, &param, &result);
  ModelOutput(result, output);
}

void PlanOut(const ApaParkingPathPlanningInputData& input,
    const ApaPlanningCalibration& calibration, ApaParkingPathPlanningOutputData& output) {
  const auto request = NativeInput(input);
  const auto param = NativeParam(input, calibration);
  ApaParkingOutPathPlanningOutput_t result{};
  ::apaParkingOutPathPlanning(&request, &param, &result);
  ModelOutput(result, output);
}

void ConnectOut(const ApaParkingPathPlanningInputData& input,
    const ApaPlanningCalibration& calibration, const ApaParkingPathPlanningOutputData& localPath,
    ApaParkingPathPlanningOutputData& output, Boolean& connected) {
  ApaParkingOutTargetConnectorInput_t request{};
  request.PlannerInput = NativeInput(input);
  request.PlannerParam = NativeParam(input, calibration);
  request.LocalPath = NativeOutput(localPath);
  ApaParkingOutTargetConnectorOutput_t result{};
  apaParkingOutTargetConnector(&request, &result);
  ModelOutput(result.ConnectedPath, output);
  connected = result.Connected;
}

void ValidatePath(const ApaParkingPathPlanningInputData& input,
    const ApaPlanningCalibration& calibration, const ApaParkingPathPlanningOutputData& candidate,
    ApaParkingPathPlanningOutputData& output, Boolean& rejected) {
  ApaParkingPathSafetyGateInput_t request{};
  request.PlannerInput = NativeInput(input);
  request.PlannerParam = NativeParam(input, calibration);
  request.CandidateOutput = NativeOutput(candidate);
  ApaParkingPathSafetyGateOutput_t result{};
  apaParkingPathSafetyGate(&request, &result);
  ModelOutput(result.ValidatedOutput, output);
  rejected = result.Rejected;
}

ApaRecoveryEnvironment ToModel(const VehicleCollisionCheckInput_t& source) {
  ApaRecoveryEnvironment value{};
  value.CheckType = source.CheckType;
  value.CollisionModel = source.CollisionModel;
  value.CollisionDemand = source.CollisionDemand;
  value.ParkingOutFlag = source.ParkingOutFlag;
  value.VerticalLastStep = source.VerticalLastStep;
  value.SlotClass = source.SlotClass;
  value.TarPosValid = source.TarPosValid;
  value.TarPosX = source.TarPosX;
  value.TarPosY = source.TarPosY;
  value.TarPosTheta = source.TarPosTheta;
  value.VehPosX = source.VehPosX;
  value.VehPosY = source.VehPosY;
  value.VehPosTheta = source.VehPosTheta;
  value.VehLF = source.VehLF;
  value.VehLB = source.VehLB;
  value.VehLL = source.VehLL;
  value.VehW = source.VehW;
  value.VehWB = source.VehWB;
  value.UseAutoMargin = source.UseAutoMargin;
  value.VehNearSlotFlag = source.VehNearSlotFlag;
  value.VehInSlotFlag = source.VehInSlotFlag;
  value.FrontSafeMargin = source.FrontSafeMargin;
  value.RearSafeMargin = source.RearSafeMargin;
  value.SideSafeMargin = source.SideSafeMargin;
  value.WhlBseLongMargin = source.WhlBseLongMargin;
  value.WhlBseLatMargin = source.WhlBseLatMargin;
  value.FrontSafeMarginNearSlotVt = source.FrontSafeMarginNearSlotVt;
  value.RearSafeMarginNearSlotVt = source.RearSafeMarginNearSlotVt;
  value.SideSafeMarginNearSlotVt = source.SideSafeMarginNearSlotVt;
  value.FrontSafeMarginAwaySlotVt = source.FrontSafeMarginAwaySlotVt;
  value.RearSafeMarginAwaySlotVt = source.RearSafeMarginAwaySlotVt;
  value.SideSafeMarginAwaySlotVt = source.SideSafeMarginAwaySlotVt;
  value.FrontSafeMarginNearSlotHt = source.FrontSafeMarginNearSlotHt;
  value.RearSafeMarginNearSlotHt = source.RearSafeMarginNearSlotHt;
  value.SideSafeMarginNearSlotHt = source.SideSafeMarginNearSlotHt;
  value.FrontSafeMarginAwaySlotHt = source.FrontSafeMarginAwaySlotHt;
  value.RearSafeMarginAwaySlotHt = source.RearSafeMarginAwaySlotHt;
  value.SideSafeMarginAwaySlotHt = source.SideSafeMarginAwaySlotHt;
  value.FrontTriLat = source.FrontTriLat;
  value.FrontTriLon = source.FrontTriLon;
  value.BackTriLat = source.BackTriLat;
  value.BackTriLon = source.BackTriLon;
  value.BoundaryLineNum = source.BoundaryLineNum;
  static_assert(std::extent_v<decltype(VehicleCollisionCheckInput_t::BoundaryLineX1)> == 64);
  for (Integer i = 0; i < 64; ++i) value.BoundaryLineX1[i] = source.BoundaryLineX1[i];
  static_assert(std::extent_v<decltype(VehicleCollisionCheckInput_t::BoundaryLineY1)> == 64);
  for (Integer i = 0; i < 64; ++i) value.BoundaryLineY1[i] = source.BoundaryLineY1[i];
  static_assert(std::extent_v<decltype(VehicleCollisionCheckInput_t::BoundaryLineX2)> == 64);
  for (Integer i = 0; i < 64; ++i) value.BoundaryLineX2[i] = source.BoundaryLineX2[i];
  static_assert(std::extent_v<decltype(VehicleCollisionCheckInput_t::BoundaryLineY2)> == 64);
  for (Integer i = 0; i < 64; ++i) value.BoundaryLineY2[i] = source.BoundaryLineY2[i];
  value.BoundarySepPtsNum = source.BoundarySepPtsNum;
  static_assert(std::extent_v<decltype(VehicleCollisionCheckInput_t::BoundarySepPtsX)> == 128);
  for (Integer i = 0; i < 128; ++i) value.BoundarySepPtsX[i] = source.BoundarySepPtsX[i];
  static_assert(std::extent_v<decltype(VehicleCollisionCheckInput_t::BoundarySepPtsY)> == 128);
  for (Integer i = 0; i < 128; ++i) value.BoundarySepPtsY[i] = source.BoundarySepPtsY[i];
  value.SlotTrackStopperNum = source.SlotTrackStopperNum;
  static_assert(std::extent_v<decltype(VehicleCollisionCheckInput_t::SlotTrackStopperX1)> == 64);
  for (Integer i = 0; i < 64; ++i) value.SlotTrackStopperX1[i] = source.SlotTrackStopperX1[i];
  static_assert(std::extent_v<decltype(VehicleCollisionCheckInput_t::SlotTrackStopperY1)> == 64);
  for (Integer i = 0; i < 64; ++i) value.SlotTrackStopperY1[i] = source.SlotTrackStopperY1[i];
  static_assert(std::extent_v<decltype(VehicleCollisionCheckInput_t::SlotTrackStopperX2)> == 64);
  for (Integer i = 0; i < 64; ++i) value.SlotTrackStopperX2[i] = source.SlotTrackStopperX2[i];
  static_assert(std::extent_v<decltype(VehicleCollisionCheckInput_t::SlotTrackStopperY2)> == 64);
  for (Integer i = 0; i < 64; ++i) value.SlotTrackStopperY2[i] = source.SlotTrackStopperY2[i];
  value.SeparateStopperNum = source.SeparateStopperNum;
  static_assert(std::extent_v<decltype(VehicleCollisionCheckInput_t::SeparateStopperX)> == 128);
  for (Integer i = 0; i < 128; ++i) value.SeparateStopperX[i] = source.SeparateStopperX[i];
  static_assert(std::extent_v<decltype(VehicleCollisionCheckInput_t::SeparateStopperY)> == 128);
  for (Integer i = 0; i < 128; ++i) value.SeparateStopperY[i] = source.SeparateStopperY[i];
  value.VirtualBoundaryLineNum = source.VirtualBoundaryLineNum;
  static_assert(std::extent_v<decltype(VehicleCollisionCheckInput_t::VirtualBoundaryLineX1)> == 64);
  for (Integer i = 0; i < 64; ++i) value.VirtualBoundaryLineX1[i] = source.VirtualBoundaryLineX1[i];
  static_assert(std::extent_v<decltype(VehicleCollisionCheckInput_t::VirtualBoundaryLineY1)> == 64);
  for (Integer i = 0; i < 64; ++i) value.VirtualBoundaryLineY1[i] = source.VirtualBoundaryLineY1[i];
  static_assert(std::extent_v<decltype(VehicleCollisionCheckInput_t::VirtualBoundaryLineX2)> == 64);
  for (Integer i = 0; i < 64; ++i) value.VirtualBoundaryLineX2[i] = source.VirtualBoundaryLineX2[i];
  static_assert(std::extent_v<decltype(VehicleCollisionCheckInput_t::VirtualBoundaryLineY2)> == 64);
  for (Integer i = 0; i < 64; ++i) value.VirtualBoundaryLineY2[i] = source.VirtualBoundaryLineY2[i];
  value.SlotPtsValid = source.SlotPtsValid;
  static_assert(std::extent_v<decltype(VehicleCollisionCheckInput_t::SlotPts)> == 8);
  for (Integer i = 0; i < 8; ++i) value.SlotPts[i] = source.SlotPts[i];
  return value;
}

VehicleCollisionCheckInput_t ToNative(const ApaRecoveryEnvironment& source) {
  VehicleCollisionCheckInput_t value{};
  value.CheckType = source.CheckType;
  value.CollisionModel = source.CollisionModel;
  value.CollisionDemand = source.CollisionDemand;
  value.ParkingOutFlag = source.ParkingOutFlag;
  value.VerticalLastStep = source.VerticalLastStep;
  value.SlotClass = source.SlotClass;
  value.TarPosValid = source.TarPosValid;
  value.TarPosX = source.TarPosX;
  value.TarPosY = source.TarPosY;
  value.TarPosTheta = source.TarPosTheta;
  value.VehPosX = source.VehPosX;
  value.VehPosY = source.VehPosY;
  value.VehPosTheta = source.VehPosTheta;
  value.VehLF = source.VehLF;
  value.VehLB = source.VehLB;
  value.VehLL = source.VehLL;
  value.VehW = source.VehW;
  value.VehWB = source.VehWB;
  value.UseAutoMargin = source.UseAutoMargin;
  value.VehNearSlotFlag = source.VehNearSlotFlag;
  value.VehInSlotFlag = source.VehInSlotFlag;
  value.FrontSafeMargin = source.FrontSafeMargin;
  value.RearSafeMargin = source.RearSafeMargin;
  value.SideSafeMargin = source.SideSafeMargin;
  value.WhlBseLongMargin = source.WhlBseLongMargin;
  value.WhlBseLatMargin = source.WhlBseLatMargin;
  value.FrontSafeMarginNearSlotVt = source.FrontSafeMarginNearSlotVt;
  value.RearSafeMarginNearSlotVt = source.RearSafeMarginNearSlotVt;
  value.SideSafeMarginNearSlotVt = source.SideSafeMarginNearSlotVt;
  value.FrontSafeMarginAwaySlotVt = source.FrontSafeMarginAwaySlotVt;
  value.RearSafeMarginAwaySlotVt = source.RearSafeMarginAwaySlotVt;
  value.SideSafeMarginAwaySlotVt = source.SideSafeMarginAwaySlotVt;
  value.FrontSafeMarginNearSlotHt = source.FrontSafeMarginNearSlotHt;
  value.RearSafeMarginNearSlotHt = source.RearSafeMarginNearSlotHt;
  value.SideSafeMarginNearSlotHt = source.SideSafeMarginNearSlotHt;
  value.FrontSafeMarginAwaySlotHt = source.FrontSafeMarginAwaySlotHt;
  value.RearSafeMarginAwaySlotHt = source.RearSafeMarginAwaySlotHt;
  value.SideSafeMarginAwaySlotHt = source.SideSafeMarginAwaySlotHt;
  value.FrontTriLat = source.FrontTriLat;
  value.FrontTriLon = source.FrontTriLon;
  value.BackTriLat = source.BackTriLat;
  value.BackTriLon = source.BackTriLon;
  value.BoundaryLineNum = source.BoundaryLineNum;
  static_assert(std::extent_v<decltype(VehicleCollisionCheckInput_t::BoundaryLineX1)> == 64);
  for (Integer i = 0; i < 64; ++i) value.BoundaryLineX1[i] = source.BoundaryLineX1[i];
  static_assert(std::extent_v<decltype(VehicleCollisionCheckInput_t::BoundaryLineY1)> == 64);
  for (Integer i = 0; i < 64; ++i) value.BoundaryLineY1[i] = source.BoundaryLineY1[i];
  static_assert(std::extent_v<decltype(VehicleCollisionCheckInput_t::BoundaryLineX2)> == 64);
  for (Integer i = 0; i < 64; ++i) value.BoundaryLineX2[i] = source.BoundaryLineX2[i];
  static_assert(std::extent_v<decltype(VehicleCollisionCheckInput_t::BoundaryLineY2)> == 64);
  for (Integer i = 0; i < 64; ++i) value.BoundaryLineY2[i] = source.BoundaryLineY2[i];
  value.BoundarySepPtsNum = source.BoundarySepPtsNum;
  static_assert(std::extent_v<decltype(VehicleCollisionCheckInput_t::BoundarySepPtsX)> == 128);
  for (Integer i = 0; i < 128; ++i) value.BoundarySepPtsX[i] = source.BoundarySepPtsX[i];
  static_assert(std::extent_v<decltype(VehicleCollisionCheckInput_t::BoundarySepPtsY)> == 128);
  for (Integer i = 0; i < 128; ++i) value.BoundarySepPtsY[i] = source.BoundarySepPtsY[i];
  value.SlotTrackStopperNum = source.SlotTrackStopperNum;
  static_assert(std::extent_v<decltype(VehicleCollisionCheckInput_t::SlotTrackStopperX1)> == 64);
  for (Integer i = 0; i < 64; ++i) value.SlotTrackStopperX1[i] = source.SlotTrackStopperX1[i];
  static_assert(std::extent_v<decltype(VehicleCollisionCheckInput_t::SlotTrackStopperY1)> == 64);
  for (Integer i = 0; i < 64; ++i) value.SlotTrackStopperY1[i] = source.SlotTrackStopperY1[i];
  static_assert(std::extent_v<decltype(VehicleCollisionCheckInput_t::SlotTrackStopperX2)> == 64);
  for (Integer i = 0; i < 64; ++i) value.SlotTrackStopperX2[i] = source.SlotTrackStopperX2[i];
  static_assert(std::extent_v<decltype(VehicleCollisionCheckInput_t::SlotTrackStopperY2)> == 64);
  for (Integer i = 0; i < 64; ++i) value.SlotTrackStopperY2[i] = source.SlotTrackStopperY2[i];
  value.SeparateStopperNum = source.SeparateStopperNum;
  static_assert(std::extent_v<decltype(VehicleCollisionCheckInput_t::SeparateStopperX)> == 128);
  for (Integer i = 0; i < 128; ++i) value.SeparateStopperX[i] = source.SeparateStopperX[i];
  static_assert(std::extent_v<decltype(VehicleCollisionCheckInput_t::SeparateStopperY)> == 128);
  for (Integer i = 0; i < 128; ++i) value.SeparateStopperY[i] = source.SeparateStopperY[i];
  value.VirtualBoundaryLineNum = source.VirtualBoundaryLineNum;
  static_assert(std::extent_v<decltype(VehicleCollisionCheckInput_t::VirtualBoundaryLineX1)> == 64);
  for (Integer i = 0; i < 64; ++i) value.VirtualBoundaryLineX1[i] = source.VirtualBoundaryLineX1[i];
  static_assert(std::extent_v<decltype(VehicleCollisionCheckInput_t::VirtualBoundaryLineY1)> == 64);
  for (Integer i = 0; i < 64; ++i) value.VirtualBoundaryLineY1[i] = source.VirtualBoundaryLineY1[i];
  static_assert(std::extent_v<decltype(VehicleCollisionCheckInput_t::VirtualBoundaryLineX2)> == 64);
  for (Integer i = 0; i < 64; ++i) value.VirtualBoundaryLineX2[i] = source.VirtualBoundaryLineX2[i];
  static_assert(std::extent_v<decltype(VehicleCollisionCheckInput_t::VirtualBoundaryLineY2)> == 64);
  for (Integer i = 0; i < 64; ++i) value.VirtualBoundaryLineY2[i] = source.VirtualBoundaryLineY2[i];
  value.SlotPtsValid = source.SlotPtsValid;
  static_assert(std::extent_v<decltype(VehicleCollisionCheckInput_t::SlotPts)> == 8);
  for (Integer i = 0; i < 8; ++i) value.SlotPts[i] = source.SlotPts[i];
  return value;
}

ApaVirtualRecoveryContext ToModel(const ApaVirtualRecoveryContext_t& source) {
  ApaVirtualRecoveryContext value{};
  value.Enabled = source.Enabled;
  value.OriginX = source.OriginX;
  value.OriginY = source.OriginY;
  value.OriginYaw = source.OriginYaw;
  value.AnchorX = source.AnchorX;
  value.AnchorY = source.AnchorY;
  value.AnchorYaw = source.AnchorYaw;
  value.MaxLength = source.MaxLength;
  value.PrefixLength = source.PrefixLength;
  value.MaxInitialDepth = source.MaxInitialDepth;
  value.RestoredClearance = source.RestoredClearance;
  value.MaxSpeed = source.MaxSpeed;
  value.MaxDuration = source.MaxDuration;
  value.LineCount = source.LineCount;
  static_assert(std::extent_v<decltype(ApaVirtualRecoveryContext_t::Allowed)> == 64);
  for (Integer i = 0; i < 64; ++i) value.Allowed[i] = source.Allowed[i];
  static_assert(std::extent_v<decltype(ApaVirtualRecoveryContext_t::Side)> == 64);
  for (Integer i = 0; i < 64; ++i) value.Side[i] = source.Side[i];
  static_assert(std::extent_v<decltype(ApaVirtualRecoveryContext_t::InitialClearance)> == 64);
  for (Integer i = 0; i < 64; ++i) value.InitialClearance[i] = source.InitialClearance[i];
  static_assert(std::extent_v<decltype(ApaVirtualRecoveryContext_t::LineX1)> == 64);
  for (Integer i = 0; i < 64; ++i) value.LineX1[i] = source.LineX1[i];
  static_assert(std::extent_v<decltype(ApaVirtualRecoveryContext_t::LineY1)> == 64);
  for (Integer i = 0; i < 64; ++i) value.LineY1[i] = source.LineY1[i];
  static_assert(std::extent_v<decltype(ApaVirtualRecoveryContext_t::LineX2)> == 64);
  for (Integer i = 0; i < 64; ++i) value.LineX2[i] = source.LineX2[i];
  static_assert(std::extent_v<decltype(ApaVirtualRecoveryContext_t::LineY2)> == 64);
  for (Integer i = 0; i < 64; ++i) value.LineY2[i] = source.LineY2[i];
  return value;
}

ApaVirtualRecoveryContext_t ToNative(const ApaVirtualRecoveryContext& source) {
  ApaVirtualRecoveryContext_t value{};
  value.Enabled = source.Enabled;
  value.OriginX = source.OriginX;
  value.OriginY = source.OriginY;
  value.OriginYaw = source.OriginYaw;
  value.AnchorX = source.AnchorX;
  value.AnchorY = source.AnchorY;
  value.AnchorYaw = source.AnchorYaw;
  value.MaxLength = source.MaxLength;
  value.PrefixLength = source.PrefixLength;
  value.MaxInitialDepth = source.MaxInitialDepth;
  value.RestoredClearance = source.RestoredClearance;
  value.MaxSpeed = source.MaxSpeed;
  value.MaxDuration = source.MaxDuration;
  value.LineCount = source.LineCount;
  static_assert(std::extent_v<decltype(ApaVirtualRecoveryContext_t::Allowed)> == 64);
  for (Integer i = 0; i < 64; ++i) value.Allowed[i] = source.Allowed[i];
  static_assert(std::extent_v<decltype(ApaVirtualRecoveryContext_t::Side)> == 64);
  for (Integer i = 0; i < 64; ++i) value.Side[i] = source.Side[i];
  static_assert(std::extent_v<decltype(ApaVirtualRecoveryContext_t::InitialClearance)> == 64);
  for (Integer i = 0; i < 64; ++i) value.InitialClearance[i] = source.InitialClearance[i];
  static_assert(std::extent_v<decltype(ApaVirtualRecoveryContext_t::LineX1)> == 64);
  for (Integer i = 0; i < 64; ++i) value.LineX1[i] = source.LineX1[i];
  static_assert(std::extent_v<decltype(ApaVirtualRecoveryContext_t::LineY1)> == 64);
  for (Integer i = 0; i < 64; ++i) value.LineY1[i] = source.LineY1[i];
  static_assert(std::extent_v<decltype(ApaVirtualRecoveryContext_t::LineX2)> == 64);
  for (Integer i = 0; i < 64; ++i) value.LineX2[i] = source.LineX2[i];
  static_assert(std::extent_v<decltype(ApaVirtualRecoveryContext_t::LineY2)> == 64);
  for (Integer i = 0; i < 64; ++i) value.LineY2[i] = source.LineY2[i];
  return value;
}

ApaVirtualRecoveryPrefix ToModel(const ApaVirtualRecoveryPrefix_t& source) {
  ApaVirtualRecoveryPrefix value{};
  value.Valid = source.Valid;
  value.Count = source.Count;
  value.Direction = source.Direction;
  value.Length = source.Length;
  value.Curvature = source.Curvature;
  static_assert(std::extent_v<decltype(ApaVirtualRecoveryPrefix_t::X)> == 32);
  for (Integer i = 0; i < 32; ++i) value.X[i] = source.X[i];
  static_assert(std::extent_v<decltype(ApaVirtualRecoveryPrefix_t::Y)> == 32);
  for (Integer i = 0; i < 32; ++i) value.Y[i] = source.Y[i];
  static_assert(std::extent_v<decltype(ApaVirtualRecoveryPrefix_t::Yaw)> == 32);
  for (Integer i = 0; i < 32; ++i) value.Yaw[i] = source.Yaw[i];
  static_assert(std::extent_v<decltype(ApaVirtualRecoveryPrefix_t::Odo)> == 32);
  for (Integer i = 0; i < 32; ++i) value.Odo[i] = source.Odo[i];
  return value;
}

ApaVirtualRecoveryPrefix_t ToNative(const ApaVirtualRecoveryPrefix& source) {
  ApaVirtualRecoveryPrefix_t value{};
  value.Valid = source.Valid;
  value.Count = source.Count;
  value.Direction = source.Direction;
  value.Length = source.Length;
  value.Curvature = source.Curvature;
  static_assert(std::extent_v<decltype(ApaVirtualRecoveryPrefix_t::X)> == 32);
  for (Integer i = 0; i < 32; ++i) value.X[i] = source.X[i];
  static_assert(std::extent_v<decltype(ApaVirtualRecoveryPrefix_t::Y)> == 32);
  for (Integer i = 0; i < 32; ++i) value.Y[i] = source.Y[i];
  static_assert(std::extent_v<decltype(ApaVirtualRecoveryPrefix_t::Yaw)> == 32);
  for (Integer i = 0; i < 32; ++i) value.Yaw[i] = source.Yaw[i];
  static_assert(std::extent_v<decltype(ApaVirtualRecoveryPrefix_t::Odo)> == 32);
  for (Integer i = 0; i < 32; ++i) value.Odo[i] = source.Odo[i];
  return value;
}

void PlanRecovery(const ApaRecoveryEnvironment& environment,
    const ApaVirtualRecoveryContext& context, Integer primitive, Real minimumRadius,
    UInteger sceneQueryId, ApaVirtualRecoveryPrefix& prefix) {
  const auto start = ToNative(environment);
  const auto certificate = ToNative(context);
  const auto query = Query(sceneQueryId);
  ApaVirtualRecoveryPrefix_t result{};
  apaVirtualRecoveryPrefixPlan(&start, &certificate, primitive, minimumRadius, &query, &result);
  prefix = ToModel(result);
}

}
