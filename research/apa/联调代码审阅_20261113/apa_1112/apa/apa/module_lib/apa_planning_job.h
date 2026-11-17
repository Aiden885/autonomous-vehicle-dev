#pragma once

#include <chrono>
#include <cstdint>
#include <string>
#include "ApaParkingPathPlanning.hpp"
#include "apa_config.pb.h"
#include "apa_clearance_recovery.h"

extern "C" {
#include "apaVirtualRecovery.h"
#include "apaMocoParkingControl.h"
}

namespace pangu::modules {

enum class ApaPlanResultCode : uint32_t {
  Success, NoSolution, SceneCapacity, RefinementLimit, UnsafeCandidate,
  StartContact, ClearanceNoSolution, ClearanceCapacity
};
struct ApaPlanResult {
  ApaPlanResultCode code = ApaPlanResultCode::NoSolution;
  ::apa::ApaParkingPathPlanningOutputData path{};
  ApaClearanceRecoveryInfo recovery{};
};

// One worker per attempt; cancellation terminates its process.
class ApaPlanningJob {
 public:
  using Clock = std::chrono::steady_clock;
  enum class State { Running, Complete, Timeout, Failed };
  ~ApaPlanningJob() { Cancel(); }
  ApaPlanningJob() = default;
  ApaPlanningJob(const ApaPlanningJob&) = delete;
  ApaPlanningJob& operator=(const ApaPlanningJob&) = delete;
  // Zero disables the wall-clock watchdog; search/capacity budgets still apply.
  void Start(const ::apa::ApaParkingPathPlanningInputData& input, const ApaInput& snapshot,
             double timeout_sec, const ApaMocoParkingControlParam_t& control);
  State Poll(Clock::time_point now);
  void Cancel();
  bool Running() const { return pid_ > 0; }
  bool Expired(Clock::time_point now) const { return Running() && watchdog_enabled_ && now >= deadline_; }
  const ApaPlanResult& Result() const { return result_; }

 private:
  int pid_ = 0, socket_ = -1;
  std::string request_;
  size_t sent_ = 0, received_ = 0;
  bool watchdog_enabled_ = false;
  Clock::time_point deadline_{};
  ApaPlanResult result_;
};

int RunApaPlannerWorker(int socket);
}
