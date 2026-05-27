#include "carla_gaasd_adapter.h"

#include <zmq.h>
#include <nlohmann/json.hpp>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cstdlib>
#include <cstring>
#include <string>
#include <thread>
#include <vector>

namespace {

using Json = nlohmann::json;

constexpr const char *kTopicPrefix = "gaasd.carla.";
constexpr const char *kProtocol = "gaasd_carla_bridge";
constexpr const char *kProtocolVersion = "0.3.0";
constexpr const char *kEgoTopic = "gaasd.carla.ego_state.v1";
constexpr const char *kObjectListTopic = "gaasd.carla.object_list.v1";
constexpr const char *kLeadTopic = "gaasd.carla.lead_vehicle.v1";
constexpr const char *kChassisTopic = "gaasd.carla.chassis_feedback.v1";
constexpr const char *kLaneTrackingTopic = "gaasd.carla.lane_tracking.v1";
constexpr const char *kControlTopic = "gaasd.carla.control_cmd.v1";

struct EgoState {
    double speed = 0.0;
    double x = 0.0;
    double y = 0.0;
    double yaw = 0.0;
    double acc = 0.0;
    double lastWallSec = 0.0;
};

struct LeadVehicle {
    double leadSpeed = 0.0;
    double distance = 1.0e6;
    double relativeSpeed = 0.0;
    double ttc = 1.0e6;
    int valid = 0;
    double lastWallSec = 0.0;
};

struct ChassisFeedback {
    double speed = 0.0;
    double steer = 0.0;
    int mode = 0;
    double lastWallSec = 0.0;
};

struct LaneTracking {
    double lateralOffset = 0.0;
    double headingError = 0.0;
    int laneId = 0;
    int roadId = 0;
    int valid = 0;
    double lastWallSec = 0.0;
};

struct ObjectState {
    int objectId = 0;
    int objectType = 0;
    double x = 0.0;
    double y = 0.0;
    double yaw = 0.0;
    double speed = 0.0;
    double vx = 0.0;
    double vy = 0.0;
    double length = 0.0;
    double width = 0.0;
    double height = 0.0;
};

struct Adapter {
    void *context = nullptr;
    void *sub = nullptr;
    void *pub = nullptr;
    bool initialized = false;
    bool shutdownRegistered = false;
    std::string inputEndpoint = "tcp://127.0.0.1:5701";
    std::string outputEndpoint = "tcp://127.0.0.1:5702";
    std::string source = "gaasd_adapter";
    std::string egoRoleName = "hero";
    std::string frameId = "gaasd_map";
    std::string coordinateFrame = "gaasd_map";
    double maxStaleSec = 1.0;
    double safetyMaxSpeed = 15.0;
    double safetyMaxSteer = 0.6;
    unsigned long long sequence = 1;
    unsigned long long commandCount = 0;
    EgoState ego;
    LeadVehicle lead;
    ChassisFeedback chassis;
    LaneTracking laneTracking;
    std::vector<ObjectState> objects;
    double objectsLastWallSec = 0.0;
};

Adapter g_adapter;

double now_sec()
{
    using Clock = std::chrono::steady_clock;
    const auto now = Clock::now().time_since_epoch();
    return std::chrono::duration<double>(now).count();
}

long long unix_ms()
{
    using Clock = std::chrono::system_clock;
    const auto now = Clock::now().time_since_epoch();
    return std::chrono::duration_cast<std::chrono::milliseconds>(now).count();
}

std::string env_string(const char *name, const char *fallback)
{
    const char *value = std::getenv(name);
    if (value == nullptr || value[0] == '\0') {
        return std::string(fallback);
    }
    return std::string(value);
}

double env_double(const char *name, double fallback)
{
    const char *value = std::getenv(name);
    if (value == nullptr || value[0] == '\0') {
        return fallback;
    }
    char *endPtr = nullptr;
    const double parsed = std::strtod(value, &endPtr);
    if (endPtr == value) {
        return fallback;
    }
    return parsed;
}

const Json *find_path(const Json &root, std::initializer_list<const char *> path)
{
    const Json *node = &root;
    for (const char *key : path) {
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

double get_double(const Json &root, std::initializer_list<const char *> path, double fallback)
{
    const Json *node = find_path(root, path);
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

int get_int(const Json &root, std::initializer_list<const char *> path, int fallback)
{
    const Json *node = find_path(root, path);
    if (node == nullptr) {
        return fallback;
    }
    try {
        if (node->is_number_integer()) {
            return node->get<int>();
        }
        if (node->is_boolean()) {
            return node->get<bool>() ? 1 : 0;
        }
        if (node->is_string()) {
            return std::stoi(node->get<std::string>());
        }
    } catch (...) {
        return fallback;
    }
    return fallback;
}

int get_bool_int(const Json &root, std::initializer_list<const char *> path, int fallback)
{
    const Json *node = find_path(root, path);
    if (node == nullptr) {
        return fallback;
    }
    try {
        if (node->is_boolean()) {
            return node->get<bool>() ? 1 : 0;
        }
        if (node->is_number_integer()) {
            return node->get<int>() != 0 ? 1 : 0;
        }
        if (node->is_string()) {
            const std::string text = node->get<std::string>();
            return (text == "true" || text == "1") ? 1 : 0;
        }
    } catch (...) {
        return fallback;
    }
    return fallback;
}

void set_linger_zero(void *socket)
{
    const int linger = 0;
    (void)zmq_setsockopt(socket, ZMQ_LINGER, &linger, sizeof(linger));
}

int subscribe(void *socket, const char *topic)
{
    return zmq_setsockopt(socket, ZMQ_SUBSCRIBE, topic, std::strlen(topic));
}

bool is_fresh(double lastWallSec)
{
    return lastWallSec > 0.0 && (now_sec() - lastWallSec) <= g_adapter.maxStaleSec;
}

void update_ego(const Json &payload)
{
    g_adapter.ego.speed = get_double(payload, {"velocity", "speed_mps"}, 0.0);
    g_adapter.ego.x = get_double(payload, {"pose", "x_m"}, 0.0);
    g_adapter.ego.y = get_double(payload, {"pose", "y_m"}, 0.0);
    g_adapter.ego.yaw = get_double(payload, {"pose", "yaw_rad"}, 0.0);
    g_adapter.ego.acc = get_double(payload, {"acceleration", "longitudinal_mps2"}, 0.0);
    g_adapter.ego.lastWallSec = now_sec();
}

void update_lead(const Json &payload)
{
    g_adapter.lead.leadSpeed = get_double(payload, {"lead_speed_mps"}, 0.0);
    g_adapter.lead.distance = get_double(payload, {"clearance_m"}, 1.0e6);
    g_adapter.lead.relativeSpeed = get_double(payload, {"relative_speed_mps"}, 0.0);
    g_adapter.lead.ttc = get_double(payload, {"ttc_sec"}, 1.0e6);
    g_adapter.lead.valid = get_bool_int(payload, {"valid"}, 0);
    g_adapter.lead.lastWallSec = now_sec();
}

void update_chassis(const Json &payload)
{
    g_adapter.chassis.speed = get_double(payload, {"speed_mps"}, 0.0);
    g_adapter.chassis.steer = get_double(payload, {"steering_angle_rad"}, 0.0);
    g_adapter.chassis.mode = get_int(payload, {"mode"}, 0);
    g_adapter.chassis.lastWallSec = now_sec();
}

void update_lane_tracking(const Json &payload)
{
    g_adapter.laneTracking.lateralOffset = get_double(payload, {"lateral_offset_m"}, 0.0);
    g_adapter.laneTracking.headingError = get_double(payload, {"heading_error_rad"}, 0.0);
    g_adapter.laneTracking.laneId = get_int(payload, {"lane_id"}, 0);
    g_adapter.laneTracking.roadId = get_int(payload, {"road_id"}, 0);
    g_adapter.laneTracking.valid = get_bool_int(payload, {"valid"}, 0);
    g_adapter.laneTracking.lastWallSec = now_sec();
}

void update_object_list(const Json &payload)
{
    g_adapter.objects.clear();
    const Json *objects = find_path(payload, {"objects"});
    if (objects == nullptr || !objects->is_array()) {
        g_adapter.objectsLastWallSec = now_sec();
        return;
    }

    const std::size_t maxObjects = 256U;
    g_adapter.objects.reserve(std::min(objects->size(), maxObjects));
    for (const Json &item : *objects) {
        if (g_adapter.objects.size() >= maxObjects) {
            break;
        }
        ObjectState object;
        object.objectId = get_int(item, {"object_id"}, 0);
        object.objectType = get_int(item, {"type_code"}, 0);
        object.x = get_double(item, {"pose", "x_m"}, 0.0);
        object.y = get_double(item, {"pose", "y_m"}, 0.0);
        object.yaw = get_double(item, {"pose", "yaw_rad"}, 0.0);
        object.speed = get_double(item, {"velocity", "speed_mps"}, 0.0);
        object.vx = get_double(item, {"velocity", "vx_mps"}, 0.0);
        object.vy = get_double(item, {"velocity", "vy_mps"}, 0.0);
        object.length = get_double(item, {"dimension", "length_m"}, 0.0);
        object.width = get_double(item, {"dimension", "width_m"}, 0.0);
        object.height = get_double(item, {"dimension", "height_m"}, 0.0);
        g_adapter.objects.push_back(object);
    }
    g_adapter.objectsLastWallSec = now_sec();
}

void update_cache(const std::string &topic, const std::string &text)
{
    try {
        const Json message = Json::parse(text);
        const Json *payload = find_path(message, {"payload"});
        if (payload == nullptr || !payload->is_object()) {
            return;
        }
        if (topic == kEgoTopic) {
            update_ego(*payload);
        } else if (topic == kObjectListTopic) {
            update_object_list(*payload);
        } else if (topic == kLeadTopic) {
            update_lead(*payload);
        } else if (topic == kChassisTopic) {
            update_chassis(*payload);
        } else if (topic == kLaneTrackingTopic) {
            update_lane_tracking(*payload);
        }
    } catch (...) {
        return;
    }
}

int recv_one_message(int flags)
{
    zmq_msg_t topicMsg;
    zmq_msg_t payloadMsg;
    if (zmq_msg_init(&topicMsg) != 0) {
        return -1;
    }
    if (zmq_msg_init(&payloadMsg) != 0) {
        zmq_msg_close(&topicMsg);
        return -1;
    }

    const int topicRc = zmq_msg_recv(&topicMsg, g_adapter.sub, flags);
    if (topicRc < 0) {
        zmq_msg_close(&payloadMsg);
        zmq_msg_close(&topicMsg);
        return 0;
    }

    const int payloadRc = zmq_msg_recv(&payloadMsg, g_adapter.sub, 0);
    if (payloadRc >= 0) {
        const std::string topic(
            static_cast<const char *>(zmq_msg_data(&topicMsg)),
            zmq_msg_size(&topicMsg));
        const std::string payload(
            static_cast<const char *>(zmq_msg_data(&payloadMsg)),
            zmq_msg_size(&payloadMsg));
        update_cache(topic, payload);
    }

    zmq_msg_close(&payloadMsg);
    zmq_msg_close(&topicMsg);
    return payloadRc >= 0 ? 1 : -1;
}

Json make_header(unsigned long long sequence)
{
    return Json{
        {"protocol", kProtocol},
        {"protocol_version", kProtocolVersion},
        {"message_type", kControlTopic},
        {"frame_id", g_adapter.frameId},
        {"sequence", sequence},
        {"sim_time_sec", 0.0},
        {"delta_time_sec", 0.0},
        {"timestamp_unix_ms", unix_ms()},
        {"source", g_adapter.source},
        {"map_name", ""},
        {"ego_role_name", g_adapter.egoRoleName},
        {"coordinate_frame", g_adapter.coordinateFrame},
    };
}

int send_control(double targetSpeed, double targetAccel, double steerRad, int enable,
                 bool includeSpeed, bool includeAccel, bool includeSteer)
{
    const unsigned long long sequence = g_adapter.sequence++;
    const double safeTarget = std::max(0.0, targetSpeed);
    Json target = Json::object();
    if (includeSpeed) {
        target["target_speed_mps"] = safeTarget;
    }
    if (includeAccel) {
        target["target_accel_mps2"] = targetAccel;
    }
    if (includeSteer) {
        target["steer_rad"] = steerRad;
    }
    const Json message{
        {"header", make_header(sequence)},
        {"payload", Json{
            {"command_id", sequence},
            {"enable", enable != 0},
            {"target", target},
            {"safety", Json{
                {"max_speed_mps", g_adapter.safetyMaxSpeed},
                {"max_abs_steer_rad", g_adapter.safetyMaxSteer},
            }},
        }},
    };
    const std::string text = message.dump(-1, ' ', false, Json::error_handler_t::replace);

    const int topicRc = zmq_send(g_adapter.pub, kControlTopic, std::strlen(kControlTopic), ZMQ_SNDMORE);
    if (topicRc < 0) {
        return -2;
    }
    const int payloadRc = zmq_send(g_adapter.pub, text.data(), text.size(), 0);
    if (payloadRc < 0) {
        return -3;
    }
    g_adapter.commandCount++;
    return 0;
}

void shutdown_adapter()
{
    if (g_adapter.pub != nullptr) {
        zmq_close(g_adapter.pub);
        g_adapter.pub = nullptr;
    }
    if (g_adapter.sub != nullptr) {
        zmq_close(g_adapter.sub);
        g_adapter.sub = nullptr;
    }
    if (g_adapter.context != nullptr) {
        zmq_ctx_term(g_adapter.context);
        g_adapter.context = nullptr;
    }
    g_adapter.initialized = false;
}

} // namespace

extern "C" int carla_adapter_init(void)
{
    if (g_adapter.initialized) {
        return 0;
    }

    g_adapter.inputEndpoint = env_string("GAASD_CARLA_INPUT_ENDPOINT", "tcp://127.0.0.1:5701");
    g_adapter.outputEndpoint = env_string("GAASD_CARLA_OUTPUT_ENDPOINT", "tcp://127.0.0.1:5702");
    g_adapter.source = env_string("GAASD_CARLA_SOURCE", "gaasd_adapter");
    g_adapter.egoRoleName = env_string("GAASD_CARLA_EGO_ROLE_NAME", "hero");
    g_adapter.frameId = env_string("GAASD_CARLA_FRAME_ID", "gaasd_map");
    g_adapter.coordinateFrame = env_string("GAASD_CARLA_COORDINATE_FRAME", "gaasd_map");
    g_adapter.maxStaleSec = env_double("GAASD_CARLA_MAX_STALE_SEC", 1.0);
    g_adapter.safetyMaxSpeed = env_double("GAASD_CARLA_SAFETY_MAX_SPEED_MPS", 15.0);
    g_adapter.safetyMaxSteer = env_double("GAASD_CARLA_SAFETY_MAX_STEER_RAD", 0.6);

    g_adapter.context = zmq_ctx_new();
    if (g_adapter.context == nullptr) {
        return -1;
    }

    g_adapter.sub = zmq_socket(g_adapter.context, ZMQ_SUB);
    g_adapter.pub = zmq_socket(g_adapter.context, ZMQ_PUB);
    if (g_adapter.sub == nullptr || g_adapter.pub == nullptr) {
        shutdown_adapter();
        return -2;
    }

    set_linger_zero(g_adapter.sub);
    set_linger_zero(g_adapter.pub);
    if (subscribe(g_adapter.sub, kEgoTopic) != 0 ||
        subscribe(g_adapter.sub, kObjectListTopic) != 0 ||
        subscribe(g_adapter.sub, kLeadTopic) != 0 ||
        subscribe(g_adapter.sub, kChassisTopic) != 0 ||
        subscribe(g_adapter.sub, kLaneTrackingTopic) != 0) {
        shutdown_adapter();
        return -3;
    }

    if (zmq_connect(g_adapter.sub, g_adapter.inputEndpoint.c_str()) != 0) {
        shutdown_adapter();
        return -4;
    }
    if (zmq_connect(g_adapter.pub, g_adapter.outputEndpoint.c_str()) != 0) {
        shutdown_adapter();
        return -5;
    }

    g_adapter.initialized = true;
    if (!g_adapter.shutdownRegistered) {
        std::atexit(shutdown_adapter);
        g_adapter.shutdownRegistered = true;
    }

    const int sleepMs = static_cast<int>(env_double("GAASD_CARLA_STARTUP_SLEEP_MS", 200.0));
    if (sleepMs > 0) {
        std::this_thread::sleep_for(std::chrono::milliseconds(sleepMs));
    }
    return 0;
}

extern "C" void carla_adapter_shutdown(void)
{
    shutdown_adapter();
}

extern "C" int carla_adapter_poll(int timeout_ms)
{
    const int initRc = carla_adapter_init();
    if (initRc != 0) {
        return initRc;
    }

    int received = 0;
    zmq_pollitem_t items[] = {{g_adapter.sub, 0, ZMQ_POLLIN, 0}};
    const long timeout = timeout_ms < 0 ? 0L : static_cast<long>(timeout_ms);
    const int pollRc = zmq_poll(items, 1, timeout);
    if (pollRc < 0) {
        return -10;
    }

    if ((items[0].revents & ZMQ_POLLIN) != 0) {
        int rc = recv_one_message(0);
        if (rc > 0) {
            received += rc;
        }
    }

    for (int i = 0; i < 100; ++i) {
        int rc = recv_one_message(ZMQ_DONTWAIT);
        if (rc <= 0) {
            break;
        }
        received += rc;
    }

    return received;
}

extern "C" int carla_adapter_read_ego_state(
    double *egoV,
    double *egoX,
    double *egoY,
    double *egoYawRad,
    double *egoAcc,
    int *valid)
{
    if (egoV == nullptr || egoX == nullptr || egoY == nullptr ||
        egoYawRad == nullptr || egoAcc == nullptr || valid == nullptr) {
        return -20;
    }
    const int pollRc = carla_adapter_poll(0);
    if (pollRc < 0) {
        return pollRc;
    }
    *egoV = g_adapter.ego.speed;
    *egoX = g_adapter.ego.x;
    *egoY = g_adapter.ego.y;
    *egoYawRad = g_adapter.ego.yaw;
    *egoAcc = g_adapter.ego.acc;
    *valid = is_fresh(g_adapter.ego.lastWallSec) ? 1 : 0;
    return 0;
}

extern "C" int carla_adapter_read_lead_vehicle(
    double *leadV,
    double *distance,
    double *relativeSpeed,
    double *ttc,
    int *valid)
{
    if (leadV == nullptr || distance == nullptr || relativeSpeed == nullptr ||
        ttc == nullptr || valid == nullptr) {
        return -30;
    }
    const int pollRc = carla_adapter_poll(0);
    if (pollRc < 0) {
        return pollRc;
    }
    *leadV = g_adapter.lead.leadSpeed;
    *distance = g_adapter.lead.distance;
    *relativeSpeed = g_adapter.lead.relativeSpeed;
    *ttc = g_adapter.lead.ttc;
    *valid = (is_fresh(g_adapter.lead.lastWallSec) && g_adapter.lead.valid != 0) ? 1 : 0;
    return 0;
}

extern "C" int carla_adapter_read_chassis_feedback(
    double *speed,
    double *steer,
    int *mode,
    int *valid)
{
    if (speed == nullptr || steer == nullptr || mode == nullptr || valid == nullptr) {
        return -40;
    }
    const int pollRc = carla_adapter_poll(0);
    if (pollRc < 0) {
        return pollRc;
    }
    *speed = g_adapter.chassis.speed;
    *steer = g_adapter.chassis.steer;
    *mode = g_adapter.chassis.mode;
    *valid = is_fresh(g_adapter.chassis.lastWallSec) ? 1 : 0;
    return 0;
}

extern "C" int carla_adapter_read_lane_tracking(
    double *lateralOffset,
    double *headingError,
    int *laneId,
    int *roadId,
    int *valid)
{
    if (lateralOffset == nullptr || headingError == nullptr ||
        laneId == nullptr || roadId == nullptr || valid == nullptr) {
        return -45;
    }
    const int pollRc = carla_adapter_poll(0);
    if (pollRc < 0) {
        return pollRc;
    }
    *lateralOffset = g_adapter.laneTracking.lateralOffset;
    *headingError = g_adapter.laneTracking.headingError;
    *laneId = g_adapter.laneTracking.laneId;
    *roadId = g_adapter.laneTracking.roadId;
    *valid = (is_fresh(g_adapter.laneTracking.lastWallSec) && g_adapter.laneTracking.valid != 0) ? 1 : 0;
    return 0;
}

extern "C" int carla_adapter_read_object_list(
    int maxObjects,
    int *objectCount,
    int *objectId,
    int *objectType,
    double *objectX,
    double *objectY,
    double *objectYawRad,
    double *objectV,
    double *objectVx,
    double *objectVy,
    double *objectLength,
    double *objectWidth,
    double *objectHeight,
    int *valid)
{
    if (maxObjects <= 0 || objectCount == nullptr || valid == nullptr) {
        return -60;
    }
    const int pollRc = carla_adapter_poll(0);
    if (pollRc < 0) {
        return pollRc;
    }

    const int available = static_cast<int>(g_adapter.objects.size());
    const int count = std::min(maxObjects, available);
    for (int i = 0; i < count; ++i) {
        const ObjectState &object = g_adapter.objects[static_cast<std::size_t>(i)];
        if (objectId != nullptr) {
            objectId[i] = object.objectId;
        }
        if (objectType != nullptr) {
            objectType[i] = object.objectType;
        }
        if (objectX != nullptr) {
            objectX[i] = object.x;
        }
        if (objectY != nullptr) {
            objectY[i] = object.y;
        }
        if (objectYawRad != nullptr) {
            objectYawRad[i] = object.yaw;
        }
        if (objectV != nullptr) {
            objectV[i] = object.speed;
        }
        if (objectVx != nullptr) {
            objectVx[i] = object.vx;
        }
        if (objectVy != nullptr) {
            objectVy[i] = object.vy;
        }
        if (objectLength != nullptr) {
            objectLength[i] = object.length;
        }
        if (objectWidth != nullptr) {
            objectWidth[i] = object.width;
        }
        if (objectHeight != nullptr) {
            objectHeight[i] = object.height;
        }
    }

    *objectCount = count;
    *valid = is_fresh(g_adapter.objectsLastWallSec) ? 1 : 0;
    return 0;
}

extern "C" int carla_adapter_publish_longitudinal_cmd(double targetSpeed, int enable)
{
    const int initRc = carla_adapter_init();
    if (initRc != 0) {
        return initRc;
    }
    return send_control(targetSpeed, 0.0, 0.0, enable, true, true, true);
}

extern "C" int carla_adapter_publish_lateral_cmd(double steerRad, int enable)
{
    const int initRc = carla_adapter_init();
    if (initRc != 0) {
        return initRc;
    }
    return send_control(0.0, 0.0, steerRad, enable, false, false, true);
}

extern "C" int carla_adapter_publish_control_cmd(
    double targetSpeed,
    double targetAccel,
    double steerRad,
    int enable)
{
    const int initRc = carla_adapter_init();
    if (initRc != 0) {
        return initRc;
    }
    return send_control(targetSpeed, targetAccel, steerRad, enable, true, true, true);
}

extern "C" int carla_adapter_get_status(
    int *initialized,
    int *egoFresh,
    int *leadFresh,
    int *chassisFresh,
    unsigned long long *commandCount)
{
    if (initialized == nullptr || egoFresh == nullptr || leadFresh == nullptr ||
        chassisFresh == nullptr || commandCount == nullptr) {
        return -50;
    }
    *initialized = g_adapter.initialized ? 1 : 0;
    *egoFresh = is_fresh(g_adapter.ego.lastWallSec) ? 1 : 0;
    *leadFresh = is_fresh(g_adapter.lead.lastWallSec) ? 1 : 0;
    *chassisFresh = is_fresh(g_adapter.chassis.lastWallSec) ? 1 : 0;
    *commandCount = g_adapter.commandCount;
    return 0;
}
