#pragma once

#include "FuncModule.hpp"

namespace control {

struct AccVehicleInputChannelTraits {
  struct Input {
    Real egoSpeed = 0.0; ///< Vehicle speed from chassis/CAN bus, m/s.
    int commandType = 0; ///< Driver ACC button command from vehicle bus.
  };
  struct Output {
    Real egoSpeed = 0.0; ///< Forwarded vehicle speed, m/s.
    int commandType = 0; ///< Forwarded ACC command type.
  };
  struct Param {};
  struct State {};
  struct Sub {};
};

class AccVehicleInputChannel : public FuncModule<AccVehicleInputChannelTraits> {
public:
  using FuncModule::FuncModule;
  void run(const Input& input, Output& output) override;
};

struct AccPerceptionInputChannelTraits {
  struct Input {
    Real leadSpeed = 0.0; ///< Lead vehicle speed from perception/scene, m/s.
    Real leadDistance = 1000000.0; ///< Lead vehicle clearance from perception/scene, m.
  };
  struct Output {
    Real leadSpeed = 0.0; ///< Forwarded lead vehicle speed, m/s.
    Real leadDistance = 1000000.0; ///< Forwarded lead vehicle clearance, m.
  };
  struct Param {};
  struct State {};
  struct Sub {};
};

class AccPerceptionInputChannel
    : public FuncModule<AccPerceptionInputChannelTraits> {
public:
  using FuncModule::FuncModule;
  void run(const Input& input, Output& output) override;
};

struct AccSystemStateClassifierTraits {
  struct Input {
    Real egoSpeed = 0.0; ///< Vehicle speed, m/s.
    int controlEnabled = 0; ///< Previous ACC control latch.
    int hasHistory = 0; ///< Whether previous cruise settings exist.
  };
  struct Output {
    int systemState = 2; ///< ACC system state: 0 active, 1 standby-history, 2 standby-new, 3 low-speed.
  };
  struct Param {
    Real vMin = 0.0; ///< Minimum speed for ACC takeover, m/s.
  };
  struct State {};
  struct Sub {};
};

class AccSystemStateClassifier
    : public FuncModule<AccSystemStateClassifierTraits> {
public:
  using FuncModule::FuncModule;
  void run(const Input& input, Output& output) override;
};

struct AccDecisionTableTraits {
  struct Input {
    int systemState = 2; ///< Current ACC system state.
    int commandType = 0; ///< Current driver command type.
  };
  struct Output {
    int decision = 8; ///< ACC decision code.
  };
  struct Param {};
  struct State {};
  struct Sub {};
};

class AccDecisionTable : public FuncModule<AccDecisionTableTraits> {
public:
  using FuncModule::FuncModule;
  void run(const Input& input, Output& output) override;
};

struct AccControlMemoryUpdateTraits {
  struct Input {
    Real egoSpeed = 0.0; ///< Vehicle speed, m/s.
    int previousControlEnabled = 0; ///< Previous ACC control latch.
    int decision = 0; ///< Current ACC decision code.
    int commandType = 0; ///< Current driver command type.
  };
  struct Output {
    int enableOutput = 0; ///< ACC enable output for this cycle.
    int nextControlEnabled = 0; ///< ACC control latch for next cycle.
  };
  struct Param {
    Real vMin = 0.0; ///< Minimum speed for ACC takeover, m/s.
  };
  struct State {};
  struct Sub {};
};

class AccControlMemoryUpdate : public FuncModule<AccControlMemoryUpdateTraits> {
public:
  using FuncModule::FuncModule;
  void run(const Input& input, Output& output) override;
};

struct AccTimeGapUpdateTraits {
  struct Input {
    Real previousTimeGap = 1.8; ///< Previous following time gap, s.
    int decision = 0; ///< Current ACC decision code.
  };
  struct Output {
    Real nextTimeGap = 1.8; ///< Updated following time gap, s.
  };
  struct Param {
    Real timeGapStep = 0.2; ///< Time gap adjustment step, s.
    Real minTimeGap = 1.0; ///< Lower bound of following time gap, s.
    Real maxTimeGap = 3.0; ///< Upper bound of following time gap, s.
  };
  struct State {};
  struct Sub {};
};

class AccTimeGapUpdate : public FuncModule<AccTimeGapUpdateTraits> {
public:
  using FuncModule::FuncModule;
  void run(const Input& input, Output& output) override;
};

struct AccMaxSpeedUpdateTraits {
  struct Input {
    Real previousMaxSpeed = 5.0; ///< Previous maximum cruise speed, m/s.
    int decision = 0; ///< Current ACC decision code.
  };
  struct Output {
    Real nextMaxSpeed = 5.0; ///< Updated maximum cruise speed, m/s.
  };
  struct Param {
    Real speedStep = 1.3889; ///< Cruise speed adjustment step, m/s.
    Real minSpeed = 0.0; ///< Lower bound of cruise speed, m/s.
    Real maxSpeedCap = 5.0; ///< Upper bound of cruise speed, m/s.
  };
  struct State {};
  struct Sub {};
};

class AccMaxSpeedUpdate : public FuncModule<AccMaxSpeedUpdateTraits> {
public:
  using FuncModule::FuncModule;
  void run(const Input& input, Output& output) override;
};

struct AccHistoryUpdateTraits {
  struct Input {
    int previousHasHistory = 0; ///< Previous history flag.
    int nextControlEnabled = 0; ///< Updated control latch.
  };
  struct Output {
    int nextHasHistory = 0; ///< Updated history flag.
  };
  struct Param {};
  struct State {};
  struct Sub {};
};

class AccHistoryUpdate : public FuncModule<AccHistoryUpdateTraits> {
public:
  using FuncModule::FuncModule;
  void run(const Input& input, Output& output) override;
};

struct AccControlStateUpdateTraits {
  struct Input {
    Real egoSpeed = 0.0; ///< Vehicle speed, m/s.
    int commandType = 0; ///< Driver ACC command type.
    int previousControlEnabled = 0; ///< Previous ACC control state memory.
    int previousHasHistory = 0; ///< Previous cruise setting history flag.
  };
  struct Output {
    int enable = 0; ///< ACC enable output for this cycle.
    int nextControlEnabled = 0; ///< ACC control state memory for next cycle.
    int nextHasHistory = 0; ///< Updated cruise setting history flag.
    int decision = 0; ///< ACC decision code.
    int systemState = 2; ///< ACC system state code.
  };
  struct Param {
    Real vMin = 0.0; ///< Minimum speed for ACC takeover, m/s.
  };
  struct State {};
  struct Sub {
    AccSystemStateClassifier systemStateClassifier; ///< 根据上周期在控记忆、历史状态和车速判断当前 ACC 系统状态。
    AccDecisionTable decisionTable; ///< 根据系统状态和驾驶员指令输出当前周期决策码。
    AccControlMemoryUpdate controlMemoryUpdate; ///< 根据启控/取消决策更新下一周期在控状态记忆，并输出当前周期 enable。
    AccHistoryUpdate historyUpdate; ///< 根据下一周期在控状态更新是否存在历史巡航设定。
  };
};

class AccControlStateUpdate : public FuncModule<AccControlStateUpdateTraits> {
public:
  using FuncModule::FuncModule;
  void run(const Input& input, Output& output) override;
};

struct AccDecisionTraits {
  struct Input {
    Real egoSpeed = 0.0; ///< Vehicle speed, m/s.
    int commandType = 0; ///< Driver ACC command type.
  };
  struct Output {
    int enable = 0; ///< ACC enable output for this cycle.
    Real timeGap = 1.8; ///< Updated following time gap, s.
    Real maxSpeed = 5.0; ///< Updated maximum cruise speed, m/s.
    int decision = 0; ///< ACC decision code.
    int systemState = 2; ///< ACC system state code.
  };
  struct Param {
    Real initialTimeGap = 1.8; ///< Initial following time gap, s.
    Real initialMaxSpeed = 5.0; ///< Initial maximum cruise speed, m/s.
    Real vMin = 0.0; ///< Minimum speed for ACC takeover, m/s.
    Real timeGapStep = 0.2; ///< Time gap adjustment step, s.
    Real minTimeGap = 1.0; ///< Lower bound of following time gap, s.
    Real maxTimeGap = 3.0; ///< Upper bound of following time gap, s.
    Real speedStep = 1.3889; ///< Cruise speed adjustment step, m/s.
    Real minSpeed = 0.0; ///< Lower bound of cruise speed, m/s.
    Real maxSpeedCap = 5.0; ///< Upper bound of cruise speed, m/s.
  };
  struct State {
    int initialized = 0; ///< Whether state has been initialized.
    int controlEnabled = 0; ///< ACC control latch.
    int hasHistory = 0; ///< Whether historical cruise settings exist.
    Real timeGap = 1.8; ///< Stored following time gap, s.
    Real maxSpeed = 5.0; ///< Stored maximum cruise speed, m/s.
    int lastDecision = 0; ///< Last non-zero non-cancel decision code.
  };
  struct Sub {
    AccControlStateUpdate controlStateUpdate; ///< ACC 在控状态更新模块，统一输出系统状态、决策码、enable 和下一周期在控记忆。
    AccTimeGapUpdate timeGapUpdate; ///< 根据增大/减小时距指令更新跟车时距。
    AccMaxSpeedUpdate maxSpeedUpdate; ///< 根据加速/减速指令更新 ACC 巡航限速。
  };
};

class AccDecision : public FuncModule<AccDecisionTraits> {
public:
  using FuncModule::FuncModule;
  void run(const Input& input, Output& output) override;
};

struct AccDesiredDistanceTraits {
  struct Input {
    Real egoSpeed = 0.0; ///< Vehicle speed, m/s.
    Real timeGap = 1.8; ///< Following time gap, s.
  };
  struct Output {
    Real desiredDistance = 5.0; ///< Desired following distance, m.
  };
  struct Param {
    Real minDistance = 5.0; ///< Minimum desired distance, m.
  };
  struct State {};
  struct Sub {};
};

class AccDesiredDistance : public FuncModule<AccDesiredDistanceTraits> {
public:
  using FuncModule::FuncModule;
  void run(const Input& input, Output& output) override;
};

struct AccDistanceErrorTraits {
  struct Input {
    Real leadDistance = 1000000.0; ///< Lead vehicle clearance, m.
    Real desiredDistance = 5.0; ///< Desired following distance, m.
  };
  struct Output {
    Real distanceError = 0.0; ///< Distance error, positive means gap is larger than desired, m.
  };
  struct Param {};
  struct State {};
  struct Sub {};
};

class AccDistanceError : public FuncModule<AccDistanceErrorTraits> {
public:
  using FuncModule::FuncModule;
  void run(const Input& input, Output& output) override;
};

struct AccRelativeSpeedTraits {
  struct Input {
    Real egoSpeed = 0.0; ///< Vehicle speed, m/s.
    Real leadSpeed = 0.0; ///< Lead vehicle speed, m/s.
  };
  struct Output {
    Real relativeSpeed = 0.0; ///< Relative speed lead minus ego, m/s.
  };
  struct Param {};
  struct State {};
  struct Sub {};
};

class AccRelativeSpeed : public FuncModule<AccRelativeSpeedTraits> {
public:
  using FuncModule::FuncModule;
  void run(const Input& input, Output& output) override;
};

struct AccRawTargetSpeedTraits {
  struct Input {
    Real leadSpeed = 0.0; ///< Lead vehicle speed, m/s.
    Real distanceError = 0.0; ///< Distance error, m.
    Real relativeSpeed = 0.0; ///< Relative speed lead minus ego, m/s.
  };
  struct Output {
    Real rawTargetSpeed = 0.0; ///< Unbounded target speed, m/s.
  };
  struct Param {
    Real distanceGain = 0.25; ///< Distance error gain.
    Real speedGain = 0.4; ///< Relative speed gain.
  };
  struct State {};
  struct Sub {};
};

class AccRawTargetSpeed : public FuncModule<AccRawTargetSpeedTraits> {
public:
  using FuncModule::FuncModule;
  void run(const Input& input, Output& output) override;
};

struct AccTargetSpeedLimitTraits {
  struct Input {
    Real rawTargetSpeed = 0.0; ///< Unbounded target speed, m/s.
    Real maxSpeed = 5.0; ///< Maximum allowed cruise speed, m/s.
  };
  struct Output {
    Real limitedTargetSpeed = 0.0; ///< Speed command after lower/upper bounds, m/s.
  };
  struct Param {};
  struct State {};
  struct Sub {};
};

class AccTargetSpeedLimit : public FuncModule<AccTargetSpeedLimitTraits> {
public:
  using FuncModule::FuncModule;
  void run(const Input& input, Output& output) override;
};

struct AccEnableGateTraits {
  struct Input {
    Real limitedTargetSpeed = 0.0; ///< Target speed before enable gate, m/s.
    int enable = 0; ///< ACC enable flag.
  };
  struct Output {
    Real targetSpeed = 0.0; ///< Final target speed command, m/s.
    int valid = 0; ///< Final command validity flag.
  };
  struct Param {};
  struct State {};
  struct Sub {};
};

class AccEnableGate : public FuncModule<AccEnableGateTraits> {
public:
  using FuncModule::FuncModule;
  void run(const Input& input, Output& output) override;
};

struct AccSpeedControlTraits {
  struct Input {
    Real egoSpeed = 0.0; ///< Vehicle speed, m/s.
    Real leadSpeed = 0.0; ///< Lead vehicle speed, m/s.
    Real leadDistance = 1000000.0; ///< Lead vehicle clearance, m.
    int enable = 0; ///< ACC enable flag.
    Real timeGap = 1.8; ///< Following time gap, s.
    Real maxSpeed = 5.0; ///< Maximum cruise speed, m/s.
  };
  struct Output {
    Real targetSpeed = 0.0; ///< Final target speed command, m/s.
    int valid = 0; ///< Final command validity flag.
    Real desiredDistance = 5.0; ///< Desired following distance, m.
    Real distanceError = 0.0; ///< Distance error, m.
    Real relativeSpeed = 0.0; ///< Relative speed lead minus ego, m/s.
  };
  struct Param {
    Real minDistance = 5.0; ///< Minimum desired distance, m.
    Real distanceGain = 0.25; ///< Distance error gain.
    Real speedGain = 0.4; ///< Relative speed gain.
  };
  struct State {};
  struct Sub {
    AccDesiredDistance desiredDistance; ///< 将自车速度和跟车时距换算为期望跟车距离。
    AccDistanceError distanceError; ///< 计算实际前车距离与期望跟车距离之间的距离误差。
    AccRelativeSpeed relativeSpeed; ///< 计算前车速度减自车速度得到的纵向相对速度。
    AccRawTargetSpeed rawTargetSpeed; ///< 根据前车速度、距离误差和相对速度计算未限幅目标速度。
    AccTargetSpeedLimit targetSpeedLimit; ///< 根据当前巡航限速对目标速度做上下限约束。
    AccEnableGate enableGate; ///< 根据 ACC 使能状态决定是否输出有效目标速度。
  };
};

class AccSpeedControl : public FuncModule<AccSpeedControlTraits> {
public:
  using FuncModule::FuncModule;
  void run(const Input& input, Output& output) override;
};

struct AccMainFlowTraits {
  struct Input {
    Real egoSpeed = 0.0; ///< Vehicle speed from chassis/CAN bus, m/s.
    Real leadSpeed = 0.0; ///< Lead vehicle speed from environment perception, m/s.
    Real leadDistance = 1000000.0; ///< Lead vehicle clearance from environment perception, m.
    int commandType = 0; ///< Driver ACC button command from vehicle bus.
  };
  struct Output {
    Real targetSpeed = 0.0; ///< Target speed command, m/s.
    bool enable = false; ///< ACC enable flag.
    bool valid = false; ///< Output validity flag.
    Real timeGap = 1.8; ///< Current following time gap, s.
    Real maxSpeed = 5.0; ///< Current maximum cruise speed, m/s.
    int decision = 0; ///< Current ACC decision code.
    int systemState = 2; ///< Current ACC system state.
  };
  struct Param {
    Real initialTimeGap = 1.8; ///< Initial following time gap, s.
    Real minDistance = 5.0; ///< Minimum desired distance, m.
    Real distanceGain = 0.25; ///< Distance error gain.
    Real speedGain = 0.4; ///< Relative speed gain.
    Real initialMaxSpeed = 5.0; ///< Initial maximum cruise speed, m/s.
    Real vMin = 0.0; ///< Minimum speed for ACC takeover, m/s.
    Real timeGapStep = 0.2; ///< Time gap adjustment step, s.
    Real minTimeGap = 1.0; ///< Lower bound of following time gap, s.
    Real maxTimeGap = 3.0; ///< Upper bound of following time gap, s.
    Real speedStep = 1.3889; ///< Cruise speed adjustment step, m/s.
    Real minSpeed = 0.0; ///< Lower bound of cruise speed, m/s.
    Real maxSpeedCap = 5.0; ///< Upper bound of cruise speed, m/s.
  };
  struct State {};
  struct Sub {
    AccVehicleInputChannel vehicleInputChannel; ///< 车上信息输入通道，汇总 CAN/车身总线中的自车速度和 ACC 物理按键指令。
    AccPerceptionInputChannel perceptionInputChannel; ///< 车外感知输入通道，汇总以太网/感知链路中的前车速度和前车距离。
    AccDecision decision; ///< ACC 决策层，负责状态判断、启停控制、时距更新和限速更新。
    AccSpeedControl speedControl; ///< ACC 控制层，负责根据前车跟踪误差计算目标车速。
  };
};

class AccMainFlow : public FuncModule<AccMainFlowTraits> {
public:
  using FuncModule::FuncModule;
  void run(const Input& input, Output& output) override;
};

} // namespace control
