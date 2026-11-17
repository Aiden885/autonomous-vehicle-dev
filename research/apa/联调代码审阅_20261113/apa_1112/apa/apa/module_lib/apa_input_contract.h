#pragma once

#include <map>
#include <set>
#include <string>
#include <tuple>
#include "apa_config.pb.h"

namespace pangu::modules {

inline constexpr const char* kApaInputProtocolVersion = "0.13.0";

bool ValidateApaFeedback(const ApaInput& input, std::string* error);
bool ValidateApaRequest(const ApaInput& input, std::string* error);
std::string ApaRequestFingerprint(const ApaInput& input);

// Rejected snapshots latch a task fault and are never committed.
class ApaInputContract {
 public:
  bool Consume(const std::string& topic, const std::string& text);
  bool RejectMessage(const std::string& topic, const std::string& reason);
  ApaInput GetInput() const { return notice_; }

 private:
  struct Task {
    std::string session;
    uint64_t id = 0;
    bool operator==(const Task& other) const { return session == other.session && id == other.id; }
    bool operator<(const Task& other) const {
      return std::tie(session, id) < std::tie(other.session, other.id);
    }
  };
  struct Request {
    Task task;
    ApaInput input;
    std::string error;
    bool valid = false;
  };
  bool IsOld(const Task& task) const;
  bool Reject(const Task& task, const std::string& reason);
  void Compose(bool feedback_valid = true);
  Request& RequestFor(const Task& task);

  bool have_feedback_ = false;
  ApaInput feedback_;
  Request current_request_;
  std::map<Task, Request> pending_requests_;
  std::set<std::string> retired_sessions_;
  std::string fault_;
  std::string unbound_error_;
  ApaInput notice_;
};

}
