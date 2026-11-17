#pragma once

#include "ApaParkingPathPlanningTypes.hpp"
#include "ApaVirtualRecoveryPrefixPlanningTypes.hpp"
#include "GlobalContext.hpp"

namespace apa::native {

/** @brief 原子边界适配声明；C ABI 和场景回调生命周期由运行层管理。 */
void PlanSlot(const ApaParkingPathPlanningInputData& input,
    const ApaPlanningCalibration& calibration, ApaParkingPathPlanningOutputData& output,
    ApaParkingPathPlanningInputData& resolvedInput);
void PlanVt(const ApaParkingPathPlanningInputData& input,
    const ApaPlanningCalibration& calibration, ApaParkingPathPlanningOutputData& output);
void PlanHt(const ApaParkingPathPlanningInputData& input,
    const ApaPlanningCalibration& calibration, ApaParkingPathPlanningOutputData& output);
void PlanDthEdVt(const ApaParkingPathPlanningInputData& input,
    const ApaPlanningCalibration& calibration, ApaParkingPathPlanningOutputData& output);
void PlanVh(const ApaParkingPathPlanningInputData& input,
    const ApaPlanningCalibration& calibration, ApaParkingPathPlanningOutputData& output);
void PlanOut(const ApaParkingPathPlanningInputData& input,
    const ApaPlanningCalibration& calibration, ApaParkingPathPlanningOutputData& output);
void ConnectOut(const ApaParkingPathPlanningInputData& input,
    const ApaPlanningCalibration& calibration, const ApaParkingPathPlanningOutputData& localPath,
    ApaParkingPathPlanningOutputData& output, Boolean& connected);
void ValidatePath(const ApaParkingPathPlanningInputData& input,
    const ApaPlanningCalibration& calibration, const ApaParkingPathPlanningOutputData& candidate,
    ApaParkingPathPlanningOutputData& output, Boolean& rejected);
void PlanRecovery(const ApaRecoveryEnvironment& environment,
    const ApaVirtualRecoveryContext& context, Integer primitive, Real minimumRadius,
    UInteger sceneQueryId, ApaVirtualRecoveryPrefix& prefix);

}
