#include "lks_zmq_bridge.h"

#include <nlohmann/json.hpp>
#include <zmq.h>

#include <chrono>
#include <cstdlib>
#include <cstring>

namespace pangu::modules {
namespace {

using Json = nlohmann::json;

constexpr const char* kEgoTopic = "gaasd.carla.ego_state.v1";
constexpr const char* kLaneTopic = "gaasd.carla.lane_tracking.v1";
constexpr const char* kDriverTopic = "gaasd.carla.driver_state.v1";
constexpr const char* kControlTopic = "gaasd.carla.control_cmd.v1";

std::string EnvString(const char* name, const char* fallback) {
  const char* value = std::getenv(name);
  return value == nullptr || value[0] == '\0' ? fallback : value;
}

const Json* FindPath(const Json& root, std::initializer_list<const char*> path) {
  const Json* node = &root;
  for (const char* key : path) {
    if (!node->is_object()) {
      return nullptr;
    }
    const auto it = node->find(key);
    if (it == node->end()) {
      return nullptr;
    }
    node = &(*it);
  }
  return node;
}

double GetDouble(const Json& root, std::initializer_list<const char*> path,
                 double fallback) {
  const Json* node = FindPath(root, path);
  if (node == nullptr) {
    return fallback;
  }
  try {
    return node->is_number() ? node->get<double>() : std::stod(node->get<std::string>());
  } catch (...) {
    return fallback;
  }
}

bool GetBool(const Json& root, std::initializer_list<const char*> path,
             bool fallback) {
  const Json* node = FindPath(root, path);
  if (node == nullptr) {
    return fallback;
  }
  try {
    if (node->is_boolean()) {
      return node->get<bool>();
    }
    if (node->is_number()) {
      return node->get<double>() != 0.0;
    }
  } catch (...) {
  }
  return fallback;
}

long long UnixMs() {
  using Clock = std::chrono::system_clock;
  return std::chrono::duration_cast<std::chrono::milliseconds>(
             Clock::now().time_since_epoch())
      .count();
}

bool Subscribe(void* socket, const char* topic) {
  return zmq_setsockopt(socket, ZMQ_SUBSCRIBE, topic, std::strlen(topic)) == 0;
}

}  // namespace

LksZmqBridge::LksZmqBridge()
    : input_endpoint_(EnvString("GAASD_CARLA_INPUT_ENDPOINT", "tcp://127.0.0.1:5701")),
      output_endpoint_(EnvString("GAASD_CARLA_OUTPUT_ENDPOINT", "tcp://127.0.0.1:5702")) {}

LksZmqBridge::~LksZmqBridge() { Shutdown(); }

bool LksZmqBridge::Init() {
  if (initialized_) {
    return true;
  }
  context_ = zmq_ctx_new();
  sub_ = context_ == nullptr ? nullptr : zmq_socket(context_, ZMQ_SUB);
  pub_ = context_ == nullptr ? nullptr : zmq_socket(context_, ZMQ_PUB);
  if (sub_ == nullptr || pub_ == nullptr) {
    Shutdown();
    return false;
  }
  int linger = 0;
  zmq_setsockopt(sub_, ZMQ_LINGER, &linger, sizeof(linger));
  zmq_setsockopt(pub_, ZMQ_LINGER, &linger, sizeof(linger));
  if (!Subscribe(sub_, kEgoTopic) || !Subscribe(sub_, kLaneTopic) ||
      !Subscribe(sub_, kDriverTopic) ||
      zmq_connect(sub_, input_endpoint_.c_str()) != 0 ||
      zmq_connect(pub_, output_endpoint_.c_str()) != 0) {
    Shutdown();
    return false;
  }
  initialized_ = true;
  return true;
}

void LksZmqBridge::Shutdown() {
  if (pub_ != nullptr) zmq_close(pub_);
  if (sub_ != nullptr) zmq_close(sub_);
  if (context_ != nullptr) zmq_ctx_term(context_);
  pub_ = nullptr;
  sub_ = nullptr;
  context_ = nullptr;
  initialized_ = false;
}

bool LksZmqBridge::Poll(int timeout_ms) {
  if (!Init()) {
    return false;
  }
  fresh_input_ = false;
  zmq_pollitem_t items[] = {{sub_, 0, ZMQ_POLLIN, 0}};
  if (zmq_poll(items, 1, timeout_ms < 0 ? 0 : timeout_ms) < 0) {
    return false;
  }
  if ((items[0].revents & ZMQ_POLLIN) == 0) {
    return true;
  }
  for (int count = 0; count < 100; ++count) {
    zmq_msg_t topic_message;
    zmq_msg_t payload_message;
    zmq_msg_init(&topic_message);
    zmq_msg_init(&payload_message);
    const int topic_rc = zmq_msg_recv(&topic_message, sub_, ZMQ_DONTWAIT);
    if (topic_rc < 0) {
      zmq_msg_close(&payload_message);
      zmq_msg_close(&topic_message);
      break;
    }
    const int payload_rc = zmq_msg_recv(&payload_message, sub_, 0);
    if (payload_rc >= 0) {
      const std::string topic(static_cast<const char*>(zmq_msg_data(&topic_message)),
                              zmq_msg_size(&topic_message));
      const std::string text(static_cast<const char*>(zmq_msg_data(&payload_message)),
                             zmq_msg_size(&payload_message));
      try {
        const Json message = Json::parse(text);
        const Json* payload = FindPath(message, {"payload"});
        if (payload != nullptr && topic == kEgoTopic) {
          state_.ego_speed_mps = GetDouble(*payload, {"velocity", "speed_mps"}, 0.0);
          state_.ego_valid = true;
          fresh_input_ = true;
        } else if (payload != nullptr && topic == kLaneTopic) {
          state_.c0_m = GetDouble(*payload, {"c0_m"}, 0.0);
          state_.c1 = GetDouble(*payload, {"c1"}, 0.0);
          state_.c2_per_m = GetDouble(*payload, {"c2_per_m"}, 0.0);
          state_.c3_per_m2 = GetDouble(*payload, {"c3_per_m2"}, 0.0);
          state_.curvature_per_m = GetDouble(*payload, {"curvature_per_m"}, 0.0);
          state_.lane_valid = GetBool(*payload, {"valid"}, false);
          fresh_input_ = true;
        } else if (payload != nullptr && topic == kDriverTopic) {
          state_.brake_pressed = GetBool(*payload, {"brake_pressed"}, false);
          state_.driver_steer_norm = GetDouble(*payload, {"driver_steer_norm"}, 0.0);
          fresh_input_ = true;
        }
      } catch (...) {
      }
    }
    zmq_msg_close(&payload_message);
    zmq_msg_close(&topic_message);
  }
  return true;
}

bool LksZmqBridge::HasFreshInput() const { return fresh_input_; }

LksZmqState LksZmqBridge::GetState() const { return state_; }

bool LksZmqBridge::PublishControl(double target_speed_mps, double steer_rad,
                                  bool enable) {
  const auto sequence = sequence_++;
  const Json message{
      {"header", {{"protocol", "gaasd_carla_bridge"},
                  {"protocol_version", "0.3.0"},
                  {"message_type", kControlTopic},
                  {"sequence", sequence},
                  {"timestamp_unix_ms", UnixMs()},
                  {"source", "pangu_lks_module"}}},
      {"payload", {{"command_id", sequence},
                   {"enable", enable},
                   {"target", {{"target_speed_mps", target_speed_mps},
                                {"target_accel_mps2", 0.0},
                                {"steer_rad", steer_rad}}},
                   {"safety", {{"max_speed_mps", target_speed_mps},
                                {"max_abs_steer_rad", 0.6}}}}}};
  const std::string text = message.dump();
  return zmq_send(pub_, kControlTopic, std::strlen(kControlTopic), ZMQ_SNDMORE) >= 0 &&
         zmq_send(pub_, text.data(), text.size(), 0) >= 0;
}

}  // namespace pangu::modules
