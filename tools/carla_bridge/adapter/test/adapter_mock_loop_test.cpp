#include "carla_gaasd_adapter.h"

#include <zmq.h>
#include <nlohmann/json.hpp>

#include <chrono>
#include <cmath>
#include <cstdlib>
#include <cstring>
#include <iostream>
#include <string>
#include <thread>

namespace {

using Json = nlohmann::json;

constexpr const char *kInputEndpoint = "tcp://127.0.0.1:15701";
constexpr const char *kOutputEndpoint = "tcp://127.0.0.1:15702";
constexpr const char *kEgoTopic = "gaasd.carla.ego_state.v1";
constexpr const char *kObjectListTopic = "gaasd.carla.object_list.v1";
constexpr const char *kLeadTopic = "gaasd.carla.lead_vehicle.v1";
constexpr const char *kChassisTopic = "gaasd.carla.chassis_feedback.v1";
constexpr const char *kLaneTrackingTopic = "gaasd.carla.lane_tracking.v1";
constexpr const char *kControlTopic = "gaasd.carla.control_cmd.v1";

void set_linger_zero(void *socket)
{
    const int linger = 0;
    (void)zmq_setsockopt(socket, ZMQ_LINGER, &linger, sizeof(linger));
}

bool near_value(double actual, double expected)
{
    return std::fabs(actual - expected) < 1.0e-6;
}

void send_json(void *pub, const char *topic, const Json &payload)
{
    const Json message{
        {"header", Json{
            {"protocol", "gaasd_carla_bridge"},
            {"protocol_version", "0.3.0"},
            {"message_type", topic},
            {"sequence", 1},
        }},
        {"payload", payload},
    };
    const std::string text = message.dump();
    (void)zmq_send(pub, topic, std::strlen(topic), ZMQ_SNDMORE);
    (void)zmq_send(pub, text.data(), text.size(), 0);
}

bool recv_control(void *sub, Json *out)
{
    zmq_pollitem_t items[] = {{sub, 0, ZMQ_POLLIN, 0}};
    const int poll_rc = zmq_poll(items, 1, 1000);
    if (poll_rc <= 0 || (items[0].revents & ZMQ_POLLIN) == 0) {
        return false;
    }

    zmq_msg_t topic_msg;
    zmq_msg_t payload_msg;
    zmq_msg_init(&topic_msg);
    zmq_msg_init(&payload_msg);
    const int topic_rc = zmq_msg_recv(&topic_msg, sub, 0);
    const int payload_rc = zmq_msg_recv(&payload_msg, sub, 0);
    bool ok = false;
    if (topic_rc >= 0 && payload_rc >= 0) {
        const std::string topic(
            static_cast<const char *>(zmq_msg_data(&topic_msg)),
            zmq_msg_size(&topic_msg));
        const std::string payload(
            static_cast<const char *>(zmq_msg_data(&payload_msg)),
            zmq_msg_size(&payload_msg));
        if (topic == kControlTopic) {
            *out = Json::parse(payload);
            ok = true;
        }
    }
    zmq_msg_close(&payload_msg);
    zmq_msg_close(&topic_msg);
    return ok;
}

} // namespace

int main()
{
    int result = 1;
    void *ctx = nullptr;
    void *mock_pub = nullptr;
    void *mock_sub = nullptr;

    setenv("GAASD_CARLA_INPUT_ENDPOINT", kInputEndpoint, 1);
    setenv("GAASD_CARLA_OUTPUT_ENDPOINT", kOutputEndpoint, 1);
    setenv("GAASD_CARLA_STARTUP_SLEEP_MS", "50", 1);
    setenv("GAASD_CARLA_MAX_STALE_SEC", "5.0", 1);

    ctx = zmq_ctx_new();
    if (ctx == nullptr) {
        std::cerr << "mock ctx failed\n";
        return 1;
    }

    mock_pub = zmq_socket(ctx, ZMQ_PUB);
    mock_sub = zmq_socket(ctx, ZMQ_SUB);
    if (mock_pub == nullptr || mock_sub == nullptr) {
        std::cerr << "mock socket failed\n";
        goto cleanup;
    }
    set_linger_zero(mock_pub);
    set_linger_zero(mock_sub);

    if (zmq_bind(mock_pub, kInputEndpoint) != 0) {
        std::cerr << "mock pub bind failed\n";
        goto cleanup;
    }
    if (zmq_setsockopt(mock_sub, ZMQ_SUBSCRIBE, kControlTopic, std::strlen(kControlTopic)) != 0) {
        std::cerr << "mock sub subscribe failed\n";
        goto cleanup;
    }
    if (zmq_bind(mock_sub, kOutputEndpoint) != 0) {
        std::cerr << "mock sub bind failed\n";
        goto cleanup;
    }

    if (carla_adapter_init() != 0) {
        std::cerr << "adapter init failed\n";
        goto cleanup;
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(500));

    for (int i = 0; i < 10; ++i) {
        send_json(mock_pub, kEgoTopic, Json{
            {"velocity", Json{{"speed_mps", 4.2}}},
            {"pose", Json{{"x_m", 11.0}, {"y_m", -2.5}, {"yaw_rad", 0.12}}},
            {"acceleration", Json{{"longitudinal_mps2", 0.3}}},
        });
        send_json(mock_pub, kLeadTopic, Json{
            {"valid", true},
            {"lead_speed_mps", 3.1},
            {"clearance_m", 22.0},
            {"relative_speed_mps", -1.1},
            {"ttc_sec", 20.0},
        });
        send_json(mock_pub, kChassisTopic, Json{
            {"speed_mps", 4.0},
            {"steering_angle_rad", 0.05},
            {"mode", 1},
        });
        send_json(mock_pub, kLaneTrackingTopic, Json{
            {"valid", true},
            {"lateral_offset_m", 0.8},
            {"heading_error_rad", -0.12},
            {"lane_id", 1},
            {"road_id", 7},
        });
        send_json(mock_pub, kObjectListTopic, Json{
            {"source_type", "ground_truth"},
            {"object_count", 1},
            {"objects", Json::array({
                Json{
                    {"object_id", 42},
                    {"type_code", 1},
                    {"pose", Json{{"x_m", 21.0}, {"y_m", 2.0}, {"yaw_rad", 0.2}}},
                    {"velocity", Json{{"speed_mps", 2.3}, {"vx_mps", 2.0}, {"vy_mps", 0.1}}},
                    {"dimension", Json{{"length_m", 4.5}, {"width_m", 1.8}, {"height_m", 1.6}}},
                },
            })},
        });
        (void)carla_adapter_poll(50);
    }

    {
        double ego_v = 0.0;
        double ego_x = 0.0;
        double ego_y = 0.0;
        double ego_yaw = 0.0;
        double ego_acc = 0.0;
        double lead_v = 0.0;
        double distance = 0.0;
        double relative_speed = 0.0;
        double ttc = 0.0;
        double chassis_speed = 0.0;
        double steer = 0.0;
        int object_count = 0;
        int object_id[2] = {0, 0};
        int object_type[2] = {0, 0};
        double object_x[2] = {0.0, 0.0};
        double object_y[2] = {0.0, 0.0};
        double object_yaw[2] = {0.0, 0.0};
        double object_v[2] = {0.0, 0.0};
        double object_vx[2] = {0.0, 0.0};
        double object_vy[2] = {0.0, 0.0};
        double object_length[2] = {0.0, 0.0};
        double object_width[2] = {0.0, 0.0};
        double object_height[2] = {0.0, 0.0};
        int valid = 0;
        int lead_valid = 0;
        int chassis_valid = 0;
        double lateral_offset = 0.0;
        double heading_error = 0.0;
        int lane_id = 0;
        int road_id = 0;
        int lane_valid = 0;
        int object_valid = 0;
        int mode = 0;

        if (carla_adapter_read_ego_state(&ego_v, &ego_x, &ego_y, &ego_yaw, &ego_acc, &valid) != 0 ||
            valid != 1 ||
            !near_value(ego_v, 4.2) ||
            !near_value(ego_x, 11.0) ||
            !near_value(ego_y, -2.5) ||
            !near_value(ego_yaw, 0.12) ||
            !near_value(ego_acc, 0.3)) {
            std::cerr << "ego read mismatch\n";
            goto cleanup;
        }

        if (carla_adapter_read_lead_vehicle(&lead_v, &distance, &relative_speed, &ttc, &lead_valid) != 0 ||
            lead_valid != 1 ||
            !near_value(lead_v, 3.1) ||
            !near_value(distance, 22.0) ||
            !near_value(relative_speed, -1.1) ||
            !near_value(ttc, 20.0)) {
            std::cerr << "lead read mismatch\n";
            goto cleanup;
        }

        if (carla_adapter_read_chassis_feedback(&chassis_speed, &steer, &mode, &chassis_valid) != 0 ||
            chassis_valid != 1 ||
            mode != 1 ||
            !near_value(chassis_speed, 4.0) ||
            !near_value(steer, 0.05)) {
            std::cerr << "chassis read mismatch\n";
            goto cleanup;
        }

        if (carla_adapter_read_lane_tracking(
                &lateral_offset,
                &heading_error,
                &lane_id,
                &road_id,
                &lane_valid) != 0 ||
            lane_valid != 1 ||
            lane_id != 1 ||
            road_id != 7 ||
            !near_value(lateral_offset, 0.8) ||
            !near_value(heading_error, -0.12)) {
            std::cerr << "lane tracking read mismatch\n";
            goto cleanup;
        }

        if (carla_adapter_read_object_list(
                2,
                &object_count,
                object_id,
                object_type,
                object_x,
                object_y,
                object_yaw,
                object_v,
                object_vx,
                object_vy,
                object_length,
                object_width,
                object_height,
                &object_valid) != 0 ||
            object_valid != 1 ||
            object_count != 1 ||
            object_id[0] != 42 ||
            object_type[0] != 1 ||
            !near_value(object_x[0], 21.0) ||
            !near_value(object_y[0], 2.0) ||
            !near_value(object_yaw[0], 0.2) ||
            !near_value(object_v[0], 2.3) ||
            !near_value(object_vx[0], 2.0) ||
            !near_value(object_vy[0], 0.1) ||
            !near_value(object_length[0], 4.5) ||
            !near_value(object_width[0], 1.8) ||
            !near_value(object_height[0], 1.6)) {
            std::cerr << "object list read mismatch\n";
            goto cleanup;
        }
    }

    {
        Json control;
        bool received = false;
        for (int i = 0; i < 10 && !received; ++i) {
            if (carla_adapter_publish_longitudinal_cmd(3.5, 1) != 0) {
                std::cerr << "publish command failed\n";
                goto cleanup;
            }
            received = recv_control(mock_sub, &control);
            if (!received) {
                std::this_thread::sleep_for(std::chrono::milliseconds(100));
            }
        }
        if (!received) {
            std::cerr << "control receive failed\n";
            goto cleanup;
        }
        const double target_speed = control["payload"]["target"]["target_speed_mps"].get<double>();
        const bool enable = control["payload"]["enable"].get<bool>();
        if (!near_value(target_speed, 3.5) || !enable) {
            std::cerr << "control payload mismatch\n";
            goto cleanup;
        }
    }

    {
        Json control;
        bool received = false;
        for (int i = 0; i < 10 && !received; ++i) {
            if (carla_adapter_publish_lateral_cmd(0.12, 1) != 0) {
                std::cerr << "publish lateral command failed\n";
                goto cleanup;
            }
            received = recv_control(mock_sub, &control);
            if (!received) {
                std::this_thread::sleep_for(std::chrono::milliseconds(100));
            }
        }
        if (!received ||
            !near_value(control["payload"]["target"]["steer_rad"].get<double>(), 0.12) ||
            control["payload"]["target"].contains("target_speed_mps")) {
            std::cerr << "lateral control payload mismatch\n";
            goto cleanup;
        }
    }

    {
        Json control;
        bool received = false;
        for (int i = 0; i < 10 && !received; ++i) {
            if (carla_adapter_publish_control_cmd(4.4, 0.3, -0.08, 1) != 0) {
                std::cerr << "publish full command failed\n";
                goto cleanup;
            }
            received = recv_control(mock_sub, &control);
            if (!received) {
                std::this_thread::sleep_for(std::chrono::milliseconds(100));
            }
        }
        if (!received ||
            !near_value(control["payload"]["target"]["target_speed_mps"].get<double>(), 4.4) ||
            !near_value(control["payload"]["target"]["target_accel_mps2"].get<double>(), 0.3) ||
            !near_value(control["payload"]["target"]["steer_rad"].get<double>(), -0.08)) {
            std::cerr << "full control payload mismatch\n";
            goto cleanup;
        }
    }

    std::cout << "mock_loop_ok\n";
    result = 0;

cleanup:
    carla_adapter_shutdown();
    if (mock_sub != nullptr) {
        zmq_close(mock_sub);
    }
    if (mock_pub != nullptr) {
        zmq_close(mock_pub);
    }
    if (ctx != nullptr) {
        zmq_ctx_term(ctx);
    }
    return result;
}
