#include "scout_base.hpp"

#include <algorithm>
#include <array>
#include <chrono>
#include <cstdint>
#include <cstring>
#include <iostream>
#include <ratio>
#include <string>
#include <thread>

#include "stopwatch.h"

namespace westonrobot
{
void ScoutBase::SendRobotCmd() { SendMotionCmd(); }

void ScoutBase::SendMotionCmd()
{
    // motion control message
    ScoutMessage m_msg;

    motion_cmd_mutex_.lock();
    m_msg.body.motion_control_msg.SteeringAngle_L = current_motion_cmd_.steering_angle & 0xff;
    m_msg.body.motion_control_msg.SteeringAngle_H = (current_motion_cmd_.steering_angle >> 8) & 0xff;
    m_msg.body.motion_control_msg.TargetSpeed = current_motion_cmd_.linear_velocity;

    m_msg.body.motion_control_msg.GearShift_Along_Cross = current_motion_cmd_.gear_shift | (current_motion_cmd_.along_enable << 4) | (current_motion_cmd_.cross_enable << 5);

    m_msg.body.motion_control_msg.SpeedGear = current_motion_cmd_.speed_gear;
    m_msg.body.motion_control_msg.LightSwitch = 0;
    m_msg.body.motion_control_msg.Sum = 0;
    motion_cmd_mutex_.unlock();

    // send to serial port
    EncodeScoutMsgToUART(&m_msg, tx_buffer_, &tx_cmd_len_);
    serial_if_->SendBytes(tx_buffer_, tx_cmd_len_);
    // std::cout << "serial cmd sent" << std::endl;
}

ScoutState ScoutBase::GetScoutState()
{
    std::lock_guard<std::mutex> guard(scout_state_mutex_);
    return scout_state_;
}

void ScoutBase::SetMotionCommand(ScoutMotionCmd<double> &cmd_)
{
    // make sure cmd thread is started before attempting to send commands
    if (!cmd_thread_started_)
        StartCmdThread();

    double linear_vel = cmd_.linear_velocity * 3.6;                // m/s--->km/h
    double steering_angle = cmd_.steering_angle / M_PI * 180 * 20; // rad--->°  20:transmission ratio
    // std::cout << steering_angle << std::endl;
    std::cout << linear_vel << std::endl;
    if (linear_vel < ScoutMotionCmd<uint16_t>::min_linear_velocity)
        linear_vel = ScoutMotionCmd<uint16_t>::min_linear_velocity;
    if (linear_vel > ScoutMotionCmd<uint16_t>::max_linear_velocity)
        linear_vel = ScoutMotionCmd<uint16_t>::max_linear_velocity;
    if (steering_angle < ScoutMotionCmd<uint16_t>::min_steering_angle)
        steering_angle = ScoutMotionCmd<uint16_t>::min_steering_angle;
    if (steering_angle > ScoutMotionCmd<uint16_t>::max_steering_angle)
        steering_angle = ScoutMotionCmd<uint16_t>::max_steering_angle;

    std::lock_guard<std::mutex> guard(motion_cmd_mutex_);
    current_motion_cmd_.linear_velocity = static_cast<uint16_t>(linear_vel * 2);
    current_motion_cmd_.steering_angle = static_cast<uint16_t>(steering_angle + 1000) * 10;
    current_motion_cmd_.speed_gear = cmd_.speed_gear;
    current_motion_cmd_.along_enable = cmd_.along_enable;
    current_motion_cmd_.cross_enable = cmd_.cross_enable;
    current_motion_cmd_.gear_shift = cmd_.gear_shift;

    FeedCmdTimeoutWatchdog();
}

void ScoutBase::ParseUARTBuffer(uint8_t *buf, const size_t bufsize, size_t bytes_received)
{
    // std::cout << "bytes received from serial: " << bytes_received << std::endl;
    // serial_parser_.PrintStatistics();
    // serial_parser_.ParseBuffer(buf, bytes_received);
    ScoutMessage status_msg;
    for (int i = 0; i < bytes_received; ++i)
    {
        if (DecodeScoutMsgFromUART(buf[i], &status_msg))
            NewStatusMsgReceivedCallback(status_msg);
    }
}

void ScoutBase::NewStatusMsgReceivedCallback(const ScoutMessage &msg)
{
    // std::cout << "new status msg received" << std::endl;
    std::lock_guard<std::mutex> guard(scout_state_mutex_);
    UpdateScoutState(msg, scout_state_);
}

void ScoutBase::UpdateScoutState(const ScoutMessage &status_msg, ScoutState &state)
{
    // std::cout << "motion control feedback received" << std::endl;
    const MotionStatusMessage &msg = status_msg.body.motion_status_msg;
    state.velocity = static_cast<uint16_t>(msg.velocity) / (2 * 3.6);
    state.steering_angle = (static_cast<uint16_t>(msg.steering_angle.low_byte) | static_cast<uint16_t>(msg.steering_angle.high_byte) << 8) / 10.0 - 1000;
    state.steering_angle = state.steering_angle / 20;
    state.mode = msg.mode;
}
} // namespace westonrobot
