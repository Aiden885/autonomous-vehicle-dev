#pragma once

#include <string>
#include <mutex>
#include <optional>
#include <cstdint>

#include "common/os_adapter/os_api.h"

#include "apa_config.pb.h"
#include "ApaParkingPathPlanning.hpp"
#include "apa_planning_job.h"
#include "apa_tracking_execution.h"
#include "apa_recovery_context.h"
#include "ApaRecoveryDecision.hpp"

extern "C" {
#include "apaMocoParkingControl.h"
#include "apaMocoActuator.h"
}

namespace pangu {
namespace modules {

/**
 * @brief APA algorithm module
 * @cn_name APA泊车算法节点
 * @granularity composite
 * @tag demo
 * @version 1.0.0
 */
class apa : public icvos::adapter::IcvosBaseModule {
 public:
  apa() : icvos::adapter::IcvosBaseModule("apa") {}
  virtual ~apa();

  virtual bool Init(const std::string& ptstr = "") override;
  virtual bool InitIO() override;
  virtual bool Proc() override;
  virtual void SetVersion() override;
  virtual bool LoadConfig(const std::string& ptstr = "") override;

  DECLARE_CALLBACK(apa_input, pangu::modules::ApaInput);
  void apa_inputCallback(const pangu::modules::ApaInput& input);

 private:
  void ResetMocoState();
  void InvalidatePath();
  void LatchInputFault(const std::string& reason);
  void BeginRecovery(const std::string& reason, const pangu::modules::ApaInput& input);
  void FailRecovery(const std::string& reason, const std::string& status = "");
  bool UpdatePlanning(const pangu::modules::ApaInput& input, ApaPlanningJob::Clock::time_point now);
  bool PollPlanningResult(const pangu::modules::ApaInput& input, ApaPlanningJob::Clock::time_point now);
  void StartPlanning(const pangu::modules::ApaInput& input);
  double RecoveryRemainingSec(ApaPlanningJob::Clock::time_point now) const;
  void FinishPlanningTime();
  void RunControlStep(const ApaSafetyScene& scene, const pangu::modules::ApaInput& input,
                      pangu::modules::ApaOutput* output);

  ApaParams apa_params_;
  ::apa::ApaParkingPathPlanningOutputData planner_output_{};
  ApaMocoParkingControlParam_t control_param_{};
  ApaMocoParkingControlParam_t task_control_param_{};
  ApaMocoParkingControlState_t control_state_{};
  ApaMocoActuatorState_t actuator_state_{};
  bool has_plan_ = false;
  int path_update_idx_ = 1;
  int last_parking_action_ = 0;
  std::mutex input_mutex_;
  std::mutex control_mutex_;
  pangu::modules::ApaInput latest_input_;
  uint64_t input_revision_ = 0;
  uint64_t processed_revision_ = 0;
  std::string session_id_;
  uint64_t task_id_ = 0;
  bool parking_enabled_ = false;
  bool have_feedback_ = false;
  uint64_t last_feedback_frame_ = 0;
  double last_feedback_time_ = 0.0;
  double last_control_time_ = -1.0;
  double control_period_sec_ = 0.02;
  uint64_t control_step_count_ = 0;
  double last_control_log_time_ = -1.0;
  double last_clock_log_time_ = -1.0;
  std::string last_clock_status_;
  enum class ReplanStage { Idle, Stopping, Planning, WaitingScene, Ready, Failed };
  ReplanStage replan_stage_ = ReplanStage::Idle;
  ApaPlanningLimits planning_limits_;
  ApaPlanningJob planning_job_;
  ApaPlanningJob::Clock::time_point recovery_started_, next_attempt_;
  // Recovery timeout covers waiting only, excluding solver and admission work.
  std::optional<ApaPlanningJob::Clock::time_point> planning_started_;
  ApaPlanningJob::Clock::duration planning_time_{};
  std::string recovery_reason_;
  ApaRecoveryContext recovery_context_;
  ApaRecoveryProgress recovery_progress_;
  ::apa::ApaRecoveryDecisionTraits::Param recovery_decision_param_{};
  ::apa::ApaRecoveryDecision recovery_decision_;
  ApaClearanceRecovery pending_clearance_{}, execution_clearance_{};
  double clearance_started_sim_time_ = -1.0;
  ApaMotionPrediction motion_prediction_{};
  ApaMotionPredictionPolicy motion_prediction_policy_{};
  ApaOnlineSafety motion_limit_;
  bool motion_history_valid_ = false;
  double motion_x_ = 0.0, motion_y_ = 0.0, motion_yaw_ = 0.0;
  uint64_t recovery_id_ = 0, path_version_ = 0;
  uint32_t planning_attempts_ = 0;
  int planner_error_type_ = 0;
  uint64_t planning_origin_scene_version_ = 0;
  std::string fault_;
  std::string input_error_;
  std::string request_fingerprint_;
  pangu::modules::ApaSafetyState safety_state_;
  pangu::modules::ApaOnlineSafety online_safety_;
  uint64_t plan_scene_version_ = 0;
  pangu::modules::ApaOutput cached_output_;
  DECLARE_PUBLISHER(apa_output, pangu::modules::ApaOutput);
  DECLARE_SUBSCRIBER(apa_input);

 private:
  REGISTER_NODE_DECLARE(apa);
};

}
}
