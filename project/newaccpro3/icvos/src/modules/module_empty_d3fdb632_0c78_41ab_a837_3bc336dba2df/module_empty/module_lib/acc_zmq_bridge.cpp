#include "acc_zmq_bridge.h"

#include <zmq.h>
#include <nlohmann/json.hpp>

#include <chrono>
#include <cstdlib>
#include <cstring>

namespace pangu::modules {
namespace {

using Json = nlohmann::json;

constexpr const char* kProtocol = "gaasd_carla_bridge";
constexpr const char* kProtocolVersion = "0.3.0";
constexpr const char* kEgoTopic = "gaasd.carla.ego_state.v1";
constexpr const char* kLeadTopic = "gaasd.carla.lead_vehicle.v1";
constexpr const char* kControlTopic = "gaasd.carla.control_cmd.v1";
constexpr const char* kDriverCommandTopic = "gaasd.carla.driver_command.v1";

std::string EnvString(const char* name, const char* fallback) {
  const char* value = std::getenv(name);
  if (value == nullptr || value[0] == '\0') {
    return std::string(fallback);
  }
  return std::string(value);
}

long long UnixMs() {
  using Clock = std::chrono::system_clock;
  const auto now = Clock::now().time_since_epoch();
  return std::chrono::duration_cast<std::chrono::milliseconds>(now).count();
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
    if (node->is_number()) {
      return node->get<double>();
    }
    if (node->is_string()) {
      return std::stod(node->get<std::string>());
    }
  } catch (...) {
    return fallback;
  }
  return fallback;
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
    if (node->is_number_integer()) {
      return node->get<int>() != 0;
    }
    if (node->is_string()) {
      const auto text = node->get<std::string>();
      return text == "true" || text == "1";
    }
  } catch (...) {
    return fallback;
  }
  return fallback;
}

int GetInt(const Json& root, std::initializer_list<const char*> path,
           int fallback) {
  const Json* node = FindPath(root, path);
  if (node == nullptr) {
    return fallback;
  }
  try {
    if (node->is_number_integer()) {
      return node->get<int>();
    }
    if (node->is_number()) {
      return static_cast<int>(node->get<double>());
    }
    if (node->is_string()) {
      return std::stoi(node->get<std::string>());
    }
  } catch (...) {
    return fallback;
  }
  return fallback;
}

void SetLingerZero(void* socket) {
  const int linger = 0;
  (void)zmq_setsockopt(socket, ZMQ_LINGER, &linger, sizeof(linger));
}

bool Subscribe(void* socket, const char* topic) {
  return zmq_setsockopt(socket, ZMQ_SUBSCRIBE, topic, std::strlen(topic)) == 0;
}

}  // namespace

AccZmqBridge::AccZmqBridge()
    : input_endpoint_(EnvString("GAASD_CARLA_INPUT_ENDPOINT", "tcp://127.0.0.1:5701")),
      output_endpoint_(EnvString("GAASD_CARLA_OUTPUT_ENDPOINT", "tcp://127.0.0.1:5702")),
      source_(EnvString("GAASD_CARLA_SOURCE", "pangu_acc_module")),
      frame_id_(EnvString("GAASD_CARLA_FRAME_ID", "gaasd_map")),
      ego_role_name_(EnvString("GAASD_CARLA_EGO_ROLE_NAME", "hero")),
      coordinate_frame_(EnvString("GAASD_CARLA_COORDINATE_FRAME", "gaasd_map")) {}

AccZmqBridge::~AccZmqBridge() { Shutdown(); }

bool AccZmqBridge::Init() {
  if (initialized_) {
    return true;
  }

  context_ = zmq_ctx_new();
  if (context_ == nullptr) {
    return false;
  }
  sub_ = zmq_socket(context_, ZMQ_SUB);
  pub_ = zmq_socket(context_, ZMQ_PUB);
  if (sub_ == nullptr || pub_ == nullptr) {
    Shutdown();
    return false;
  }

  SetLingerZero(sub_);
  SetLingerZero(pub_);
  if (!Subscribe(sub_, kEgoTopic) || !Subscribe(sub_, kLeadTopic) ||
      !Subscribe(sub_, kDriverCommandTopic)) {
    Shutdown();
    return false;
  }
  if (zmq_connect(sub_, input_endpoint_.c_str()) != 0 ||
      zmq_connect(pub_, output_endpoint_.c_str()) != 0) {
    Shutdown();
    return false;
  }

  initialized_ = true;
  return true;
}

void AccZmqBridge::Shutdown() {
  if (pub_ != nullptr) {
    zmq_close(pub_);
    pub_ = nullptr;
  }
  if (sub_ != nullptr) {
    zmq_close(sub_);
    sub_ = nullptr;
  }
  if (context_ != nullptr) {
    zmq_ctx_term(context_);
    context_ = nullptr;
  }
  initialized_ = false;
}

bool AccZmqBridge::Poll(int timeout_ms) {
  if (!Init()) {
    return false;
  }

  fresh_input_ = false;
  state_.command_type = 0;
  state_.command_valid = false;
  zmq_pollitem_t items[] = {{sub_, 0, ZMQ_POLLIN, 0}};
  const int poll_rc = zmq_poll(items, 1, timeout_ms < 0 ? 0 : timeout_ms);
  if (poll_rc < 0) {
    return false;
  }
  if ((items[0].revents & ZMQ_POLLIN) == 0) {
    return true;
  }

  for (int i = 0; i < 100; ++i) {
    zmq_msg_t topic_msg;
    zmq_msg_t payload_msg;
    zmq_msg_init(&topic_msg);
    zmq_msg_init(&payload_msg);
    const int topic_rc = zmq_msg_recv(&topic_msg, sub_, ZMQ_DONTWAIT);
    if (topic_rc < 0) {
      zmq_msg_close(&payload_msg);
      zmq_msg_close(&topic_msg);
      break;
    }
    const int payload_rc = zmq_msg_recv(&payload_msg, sub_, 0);
    if (payload_rc >= 0) {
      const std::string topic(static_cast<const char*>(zmq_msg_data(&topic_msg)),
                              zmq_msg_size(&topic_msg));
      const std::string text(static_cast<const char*>(zmq_msg_data(&payload_msg)),
                             zmq_msg_size(&payload_msg));
      try {
        const Json message = Json::parse(text);
        const Json* payload = FindPath(message, {"payload"});
        if (payload != nullptr && payload->is_object()) {
          if (topic == kEgoTopic) {
            state_.ego_speed_mps =
                GetDouble(*payload, {"velocity", "speed_mps"}, 0.0);
            state_.ego_valid = true;
            fresh_input_ = true;
          } else if (topic == kLeadTopic) {
            state_.lead_speed_mps =
                GetDouble(*payload, {"lead_speed_mps"}, 0.0);
            state_.lead_distance_m =
                GetDouble(*payload, {"clearance_m"}, 1000000.0);
            state_.relative_speed_mps =
                GetDouble(*payload, {"relative_speed_mps"}, 0.0);
            state_.ttc_sec = GetDouble(*payload, {"ttc_sec"}, 1000000.0);
            state_.lead_valid = GetBool(*payload, {"valid"}, false);
            fresh_input_ = true;
          } else if (topic == kDriverCommandTopic) {
            const int command_type = GetInt(*payload, {"command_type"}, 0);
            const bool active = GetBool(*payload, {"active"}, command_type != 0);
            if (command_type >= 0 && command_type <= 7) {
              state_.command_type = active ? command_type : 0;
              state_.command_valid = true;
              fresh_input_ = true;
            }
          }
        }
      } catch (...) {
      }
    }
    zmq_msg_close(&payload_msg);
    zmq_msg_close(&topic_msg);
  }
  return true;
}

bool AccZmqBridge::HasFreshInput() const { return fresh_input_; }

AccZmqState AccZmqBridge::GetState() const { return state_; }

bool AccZmqBridge::PublishControl(double target_speed_mps, int enable) {
  if (!Init()) {
    return false;
  }

  const auto sequence = sequence_++;
  const Json message{
      {"header",
       Json{{"protocol", kProtocol},
            {"protocol_version", kProtocolVersion},
            {"message_type", kControlTopic},
            {"frame_id", frame_id_},
            {"sequence", sequence},
            {"sim_time_sec", 0.0},
            {"delta_time_sec", 0.0},
            {"timestamp_unix_ms", UnixMs()},
            {"source", source_},
            {"map_name", ""},
            {"ego_role_name", ego_role_name_},
            {"coordinate_frame", coordinate_frame_}}},
      {"payload",
       Json{{"command_id", sequence},
            {"enable", enable != 0},
            {"target",
             Json{{"target_speed_mps", target_speed_mps},
                  {"target_accel_mps2", 0.0},
                  {"steer_rad", 0.0}}},
            {"safety",
             Json{{"max_speed_mps", 15.0}, {"max_abs_steer_rad", 0.6}}}}}};
  const std::string text = message.dump(-1, ' ', false, Json::error_handler_t::replace);

  if (zmq_send(pub_, kControlTopic, std::strlen(kControlTopic), ZMQ_SNDMORE) < 0) {
    return false;
  }
  return zmq_send(pub_, text.data(), text.size(), 0) >= 0;
}

}  // namespace pangu::modules
