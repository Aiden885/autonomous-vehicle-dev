#include "apa_planning_job.h"
#include "apa_online_safety.h"
#include "apa_tracking_execution.h"
#include "apa_scene_limits.h"
#include "apa_native_planning.h"
#include "apaVehicleDefaults.h"

#include <algorithm>
#include <cerrno>
#include <csignal>
#include <cmath>
#include <cstdio>
#include <dlfcn.h>
#include <filesystem>
#include <fcntl.h>
#include <spawn.h>
#include <stdexcept>
#include <sys/socket.h>
#include <sys/time.h>
#include <sys/wait.h>
#include <type_traits>
#include <utility>
#include <vector>
#include <unistd.h>

extern "C" {
#include "apa_algorithm/apa/apaDebugTrace.h"
}

extern char** environ;
namespace pangu::modules {
namespace {
using Input = ::apa::ApaParkingPathPlanningInputData;
struct Header {
  uint32_t version = 13, input_size = sizeof(Input), result_size = sizeof(ApaPlanResult),
      control_size = sizeof(ApaMocoParkingControlParam_t),
      snapshot_size = 0, timeout_ms = 0;
};
static_assert(std::is_trivially_copyable_v<Input> && std::is_trivially_copyable_v<ApaPlanResult> &&
    std::is_trivially_copyable_v<ApaMocoParkingControlParam_t>);

std::string WorkerPath() {
  Dl_info location{};
  if (!dladdr(reinterpret_cast<void*>(&WorkerPath), &location)) throw std::runtime_error("cannot locate APA core library");
  return (std::filesystem::path(location.dli_fname).parent_path().parent_path()/"bin/apa_planner_worker").string();
}

bool Transfer(int socket, void* data, size_t size, bool writing) {
  auto* bytes = static_cast<char*>(data);
  while (size) {
    const ssize_t count = writing ? send(socket, bytes, size, MSG_NOSIGNAL) : recv(socket, bytes, size, 0);
    if (count < 0 && errno == EINTR) continue;
    if (count <= 0) return false;
    bytes += count; size -= count;
  }
  return true;
}

}

void ApaPlanningJob::Start(const Input& input, const ApaInput& snapshot, double timeout_sec,
    const ApaMocoParkingControlParam_t& control) {
  if (input.sceneQueryId != 0) throw std::invalid_argument("APA scene resources cannot cross worker IPC");
  if (!std::isfinite(timeout_sec) || timeout_sec < 0.0 || timeout_sec > 3600.0)
    throw std::invalid_argument("APA planner watchdog must be zero or within (0, 3600] seconds");
  Cancel();
  watchdog_enabled_ = timeout_sec > 0.0;
  deadline_ = Clock::now() + std::chrono::duration_cast<Clock::duration>(std::chrono::duration<double>(timeout_sec));

  const auto snapshot_bytes = snapshot.SerializeAsString();
  if (snapshot_bytes.size() > kApaSceneMaxMessageBytes ||
      snapshot.scene().obstacles_size() > kApaSceneMaxObstacles)
    throw std::invalid_argument("APA planning snapshot exceeds scene capacity");
  Header header; header.snapshot_size = snapshot_bytes.size(); header.timeout_ms = std::ceil(timeout_sec*1000.0);
  request_.assign(reinterpret_cast<const char*>(&header), sizeof(header));
  request_.append(reinterpret_cast<const char*>(&input), sizeof(input));
  request_.append(reinterpret_cast<const char*>(&control), sizeof(control));
  request_ += snapshot_bytes;
  sent_ = received_ = 0; result_ = {};
  const auto executable = WorkerPath();
  const auto parent = std::to_string(getpid());
  int sockets[2];
  if (socketpair(AF_UNIX, SOCK_STREAM|SOCK_CLOEXEC, 0, sockets) != 0) throw std::runtime_error("planner socketpair failed");
  posix_spawn_file_actions_t actions;
  posix_spawn_file_actions_init(&actions);
  posix_spawn_file_actions_adddup2(&actions, sockets[1], 3);
  if (sockets[0] != 3) posix_spawn_file_actions_addclose(&actions, sockets[0]);
  if (sockets[1] != 3) posix_spawn_file_actions_addclose(&actions, sockets[1]);
  char* argv[] = {const_cast<char*>(executable.c_str()), const_cast<char*>(parent.c_str()), nullptr};
  pid_t child = 0;
  const int error = posix_spawn(&child, executable.c_str(), &actions, nullptr, argv, environ);
  posix_spawn_file_actions_destroy(&actions);
  close(sockets[1]);
  if (error != 0) { close(sockets[0]); throw std::runtime_error("cannot start apa_planner_worker: " + std::to_string(error)); }
  pid_ = child; socket_ = sockets[0];
  if (fcntl(socket_, F_SETFL, O_NONBLOCK) != 0) { Cancel(); throw std::runtime_error("planner nonblocking socket failed"); }
}

void ApaPlanningJob::Cancel() {
  if (pid_ > 0) {
    kill(pid_, SIGKILL);
    while (waitpid(pid_, nullptr, 0) < 0 && errno == EINTR) {}
    pid_ = 0;
  }
  if (socket_ >= 0) { close(socket_); socket_ = -1; }
}

ApaPlanningJob::State ApaPlanningJob::Poll(Clock::time_point now) {
  if (Expired(now)) { Cancel(); return State::Timeout; }
  while (sent_ < request_.size()) {
    const auto count = send(socket_, request_.data()+sent_, request_.size()-sent_, MSG_DONTWAIT|MSG_NOSIGNAL);
    if (count < 0 && errno == EINTR) continue;
    if (count < 0 && (errno == EAGAIN || errno == EWOULDBLOCK)) break;
    if (count <= 0) { Cancel(); return State::Failed; }
    sent_ += count;
  }
  while (received_ < sizeof(result_)) {
    const auto count = recv(socket_, reinterpret_cast<char*>(&result_)+received_, sizeof(result_)-received_, MSG_DONTWAIT);
    if (count < 0 && errno == EINTR) continue;
    if (count < 0 && (errno == EAGAIN || errno == EWOULDBLOCK)) break;
    if (count < 0) { Cancel(); return State::Failed; }
    if (count == 0) break;
    received_ += count;
  }
  int status = 0;
  const auto exited = waitpid(pid_, &status, WNOHANG);
  if (exited == 0 || (exited < 0 && errno == EINTR)) return State::Running;
  pid_ = 0; close(socket_); socket_ = -1;
  if (exited > 0 && WIFSIGNALED(status) && WTERMSIG(status) == SIGALRM) return State::Timeout;
  return exited > 0 && WIFEXITED(status) && WEXITSTATUS(status) == 0 && received_ == sizeof(result_)
      ? State::Complete : State::Failed;
}

struct ExecutionContext {
  const ApaInput& input;
  const ApaMocoParkingControlParam_t& calibration;
  const ApaSafetyScene scene;
  ExecutionContext(const ApaInput& snapshot, const ApaMocoParkingControlParam_t& control)
      : input(snapshot), calibration(control), scene(snapshot) {}
  static Boolean Check(Integer count, const Real* x, const Real* y,
      const Real* yaw, const Real* odometer, void* data) {
    const auto& context = *static_cast<const ExecutionContext*>(data);
    ::apa::ApaParkingPathPlanningOutputData path{};
    path.planOk = true; path.pathPointCount = count;
    int previous = 0;
    for (int i=0; i<count; ++i) {
      path.pathPointX[i]=x[i]; path.pathPointY[i]=y[i];
      path.pathPointTheta[i]=yaw[i]; path.pathPointOdometer[i]=odometer[i];
      if (i+1<count && std::fabs(odometer[i])>1.e-6f) {
        const int direction = odometer[i]>0 ? 1 : -1;
        if (previous && direction!=previous) ++path.directionChangeCount;
        previous=direction; path.pathLength+=std::fabs(odometer[i]);
      }
    }

    const auto admission=CheckApaCandidatePath(context.input,path,nullptr,
        context.calibration.Chart.GoalPositionTolerance,context.calibration.Chart.GoalYawTolerance);
    if (admission.blocked()) {
      std::fprintf(stderr,"[APA candidate] stage=ranking obstacle=%s reason=%s distance=%.3f\n",
          admission.obstacle_id().c_str(),admission.reason().c_str(),admission.checked_distance_m());
      return false;
    }
    return CheckApaTrackingExecution(context.scene,context.input,path,context.calibration);
  }
};

static ApaPlanResult PlanRegularPath(Input input, const ApaInput& scene_input,
    const ApaMocoParkingControlParam_t& control) {
  ApaPlanResult result;
  ApaPlannerScene planning_scene(scene_input);

  if (input.parkingAction == APA_SCENARIO_PARKING_ACTION_PARKIN &&
      input.targetSlotKind == APA_SCENARIO_SLOT_KIND_HT &&
      scene_input.has_recovery() && scene_input.recovery().recovery_id() > 1)
    input.slotState = APA_SLOT_STATUS_TARGET_SLOT_REPLAN;
  ExecutionContext execution{scene_input,control};
  auto query = planning_scene.query();
  query.CheckExecution = &ExecutionContext::Check;
  query.ExecutionData = &execution;
  ::apa::native::SceneScope scene_scope(query);
  input.sceneQueryId = scene_scope.id();
  std::fprintf(stderr, "[APA scene] stage=worker scene=%llu obstacles=%d direct_query=1 slot_state=%ld\n",
      static_cast<unsigned long long>(scene_input.scene().version()),
      scene_input.scene().obstacles_size(), static_cast<long>(input.slotState));
  const auto start = execution.scene.CheckPlanningStart(scene_input);
  if (start.blocked()) {
    result.code = ApaPlanResultCode::UnsafeCandidate;
    result.path.errorType = APA_FAULT_COLLISION_RISK_AT_NEW_OBS;
    std::fprintf(stderr, "[APA candidate] stage=start obstacle=%s reason=%s\n",
        start.obstacle_id().c_str(), start.reason().c_str());
    return result;
  }
  ::apa::ApaParkingPathPlanning planner;
  ::apaDebugTraceBeginFrame();
  ::apa::ApaParkingPathPlanningTraits::Input model_input{input};
  ::apa::ApaParkingPathPlanningTraits::Output model_output{};
  planner.run(model_input, model_output);
  result.path = model_output.candidatePath;
  std::fprintf(stderr,
      "[APA scene] stage=search queries=%zu hits=%zu last_obstacle=%s\n",
      planning_scene.pose_queries(), planning_scene.obstacle_hits(),
      planning_scene.last_obstacle_id().c_str());
  if (!result.path.planOk) {
    result.code = ApaPlanResultCode::NoSolution;
  } else {
    const auto check = CheckApaCandidatePath(scene_input, result.path, nullptr,
        input.goalPositionTolerance, input.goalYawTolerance);
    result.code = check.blocked() ? ApaPlanResultCode::UnsafeCandidate : ApaPlanResultCode::Success;
    if (check.blocked()) std::fprintf(stderr,
        "[APA candidate] stage=worker scene=%llu obstacle=%s reason=%s distance=%.3f\n",
        static_cast<unsigned long long>(check.scene_version()), check.obstacle_id().c_str(),
        check.reason().c_str(), check.checked_distance_m());
  }
  if (result.code != ApaPlanResultCode::Success) {
    auto error_type = result.path.errorType;
    if (error_type == APA_FAULT_NO_ERR || error_type == APA_FAULT_UNKNOWN_ERR) {
      if (result.code == ApaPlanResultCode::UnsafeCandidate)
        error_type = APA_FAULT_COLLISION_RISK_AT_NEW_OBS;
    }
    result.path = {};
    result.path.errorType = error_type;
  }
  return result;
}

static bool JoinRecoveryPrefix(const ::apa::ApaParkingPathPlanningOutputData& prefix,
    const ::apa::ApaParkingPathPlanningOutputData& continuation,
    const ApaInput& origin, ::apa::ApaParkingPathPlanningOutputData* joined) {
  const int offset = prefix.pathPointCount - 1;
  const int count = offset + continuation.pathPointCount;

  if (count > 100) return false;
  *joined = continuation;
  for (int i = 0; i < continuation.pathPointCount; ++i) {
    joined->pathPointX[offset+i] = continuation.pathPointX[i];
    joined->pathPointY[offset+i] = continuation.pathPointY[i];
    joined->pathPointTheta[offset+i] = continuation.pathPointTheta[i];
    joined->pathPointOdometer[offset+i] = continuation.pathPointOdometer[i];
  }
  for (int i = 0; i < offset; ++i) {
    joined->pathPointX[i] = prefix.pathPointX[i];
    joined->pathPointY[i] = prefix.pathPointY[i];
    joined->pathPointTheta[i] = prefix.pathPointTheta[i];
    joined->pathPointOdometer[i] = prefix.pathPointOdometer[i];
  }
  joined->pathPointCount = count;
  joined->pathLength += prefix.pathLength;
  ++joined->segmentCount;
  const bool reverse_continuation = continuation.pathPointOdometer[0] < 0.0f;
  if ((prefix.pathPointOdometer[0] < 0) != reverse_continuation) ++joined->directionChangeCount;
  ++joined->maneuverStepCount;
  if (joined->secondStageEndPointOrdinal > 0) joined->secondStageEndPointOrdinal += offset;
  joined->resolvedScene.vehicleX = origin.vehicle_x();
  joined->resolvedScene.vehicleY = origin.vehicle_y();
  joined->resolvedScene.vehicleYaw = origin.vehicle_yaw();
  joined->firstForwardPointVehicleX = joined->firstForwardPointVehicleY =
      joined->firstForwardPointVehicleTheta = 0.0f;
  for (int i = 0; i + 1 < count; ++i) {
    if (joined->pathPointOdometer[i] <= 0.0f) continue;
    const double dx = joined->pathPointX[i+1] - origin.vehicle_x();
    const double dy = joined->pathPointY[i+1] - origin.vehicle_y();
    joined->firstForwardPointVehicleX = dx*std::cos(origin.vehicle_yaw()) + dy*std::sin(origin.vehicle_yaw());
    joined->firstForwardPointVehicleY = -dx*std::sin(origin.vehicle_yaw()) + dy*std::cos(origin.vehicle_yaw());
    const double yaw = joined->pathPointTheta[i+1] - origin.vehicle_yaw();
    joined->firstForwardPointVehicleTheta = std::atan2(std::sin(yaw), std::cos(yaw));
    break;
  }
  return true;
}

static ApaPlanResult PlanClearanceRecovery(const Input& input, const ApaInput& snapshot,
    const ApaClearanceRecovery& recovery, const ApaMocoParkingControlParam_t& control) {
  ApaPlanResult result;
  result.code = ApaPlanResultCode::ClearanceNoSolution;
  if (!recovery.info.enabled || !std::isfinite(input.minimumTurningRadius) || input.minimumTurningRadius <= 0.0)
    return result;

  struct Prefix {
    ::apa::ApaParkingPathPlanningOutputData path{};
    bool target_reached=false;
    double gap=0.0, effort=0.0;
    double steering=0.0;
  };
  std::vector<Prefix> prefixes;
  for (int direction : {1, -1}) {
    for (double steering : {0.0, -0.5, 0.5, -1.0, 1.0}) {
      Prefix prefix;
      if (BuildApaClearancePrefix(snapshot, recovery, steering/input.minimumTurningRadius, direction,
          &prefix.path,&prefix.target_reached,&prefix.gap)) {
        prefix.steering=std::atan(control.Lat.Wheelbase*steering/input.minimumTurningRadius);
        prefix.effort=prefix.path.pathLength+input.minimumTurningRadius*
            std::fabs(prefix.steering-snapshot.vehicle_steering_command_rad());
        prefixes.push_back(prefix);
      }
    }
  }
  std::stable_sort(prefixes.begin(), prefixes.end(), [](const auto& a, const auto& b) {
    if (a.target_reached!=b.target_reached) return a.target_reached;
    if (!a.target_reached && a.gap!=b.gap) return a.gap>b.gap;
    return a.effort<b.effort;
  });
  std::fprintf(stderr, "[APA clearance recovery] deficits=%zu valid_prefixes=%zu\n", recovery.limits.size(), prefixes.size());
  for (const auto& candidate : prefixes) {
    const auto& prefix=candidate.path;
    const int last = prefix.pathPointCount-1;
    auto continued_input = input;
    continued_input.vehicleX = prefix.pathPointX[last];
    continued_input.vehicleY = prefix.pathPointY[last];
    continued_input.vehicleYaw = prefix.pathPointTheta[last];
    auto continued_snapshot = snapshot;
    continued_snapshot.set_vehicle_x(continued_input.vehicleX);
    continued_snapshot.set_vehicle_y(continued_input.vehicleY);
    continued_snapshot.set_vehicle_yaw(continued_input.vehicleYaw);
    continued_snapshot.set_vehicle_steering_command_rad(candidate.steering);
    const auto continuation = PlanRegularPath(continued_input, continued_snapshot, control);
    if (continuation.code != ApaPlanResultCode::Success) {
      result.path.errorType = continuation.path.errorType;
      continue;
    }
    ::apa::ApaParkingPathPlanningOutputData joined{};
    if (!JoinRecoveryPrefix(prefix, continuation.path, snapshot, &joined)) {
      result.code = ApaPlanResultCode::ClearanceCapacity;
      continue;
    }
    auto admitted = recovery;
    admitted.info.prefix_length = prefix.pathLength;
    const auto check = CheckApaCandidatePath(snapshot, joined, &admitted,
        input.goalPositionTolerance, input.goalYawTolerance);
    if (check.blocked()) {
      std::fprintf(stderr, "[APA clearance recovery] rejected=%s obstacle=%s\n",
          check.reason().c_str(), check.obstacle_id().c_str());
      continue;
    }
    if (!CheckApaTrackingExecution(snapshot,joined,control,&admitted)) continue;
    result.code = ApaPlanResultCode::Success;
    result.path = joined;
    result.recovery = admitted.info;
    std::fprintf(stderr, "[APA clearance recovery] prefix=%.3f gap=%.3f steering=%.3f "
        "effort=%.3f total=%.3f points=%d accepted\n",
        prefix.pathLength,candidate.gap,candidate.steering,candidate.effort,
        joined.pathLength,joined.pathPointCount);
    return result;
  }
  result.path.planOk = false;
  result.path.errorType = APA_FAULT_COLLISION_RISK_AT_NEW_OBS;
  return result;
}

int RunApaPlannerWorker(int socket) {
  Header header;
  if (!Transfer(socket, &header, sizeof(header), false) || header.version != 13 ||
      header.control_size != sizeof(ApaMocoParkingControlParam_t) ||
      header.input_size != sizeof(Input) || header.result_size != sizeof(ApaPlanResult) ||
      header.snapshot_size > kApaSceneMaxMessageBytes ||
      header.timeout_ms > 3600000) return 2;

  itimerval timer{};
  timer.it_value.tv_sec = header.timeout_ms/1000;
  timer.it_value.tv_usec = (header.timeout_ms%1000)*1000;
  if (setitimer(ITIMER_REAL, &timer, nullptr) != 0) return 2;
  Input input{};
  ApaMocoParkingControlParam_t control{};
  std::string snapshot_bytes(header.snapshot_size, '\0');
  ApaInput scene_input;
  if (!Transfer(socket, &input, sizeof(input), false) || input.sceneQueryId != 0 ||
      !Transfer(socket, &control, sizeof(control), false) ||
      !Transfer(socket, snapshot_bytes.data(), snapshot_bytes.size(), false) ||
      !scene_input.ParseFromString(snapshot_bytes)) return 2;
  const double steering_limit = std::min(control.Lat.SteerUpperLimit, -control.Lat.SteerLowerLimit);
  const double control_radius = control.Lat.Wheelbase / std::tan(steering_limit);
  input.minimumTurningRadius = std::max({double(input.minimumTurningRadius),
      double(APA_VEHICLE_MIN_TURN_RADIUS_M), control_radius});
  ApaPlanResult result;
  const auto body = CheckApaBody(scene_input);
  if (body.blocked()) {
    result.code = ApaPlanResultCode::StartContact;
    result.path.errorType = APA_FAULT_COLLISION_RISK_AT_NEW_OBS;
    std::fprintf(stderr, "[APA candidate] stage=start obstacle=%s reason=%s\n",
        body.obstacle_id().c_str(), body.reason().c_str());
  } else {
    const auto recovery = PrepareApaClearanceRecovery(scene_input);
    result = recovery.info.enabled ? PlanClearanceRecovery(input, scene_input, recovery, control)
                                   : PlanRegularPath(input, scene_input, control);
    std::fprintf(stderr, "[APA planning radius] selected=%.6f\n",
        static_cast<double>(input.minimumTurningRadius));
  }
  return Transfer(socket, &result, sizeof(result), true) ? 0 : 2;
}
}
