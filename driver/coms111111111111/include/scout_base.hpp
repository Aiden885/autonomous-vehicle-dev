/*
 * scout_base.hpp
 *
 * Created on: Jun 04, 2019 01:22
 * Description:
 *
 * Copyright (c) 2019 Ruixiang Du (rdu)
 */

#ifndef SCOUT_BASE_HPP
#define SCOUT_BASE_HPP

#include <cstdint>
#include <functional>
#include <mutex>
#include <string>
#include <thread>

#include "mobile_base.hpp"

#include "scout_protocol.h"
#include "scout_types.hpp"
#include "scout_uart_parser.h"

namespace westonrobot
{
class ScoutBase : public MobileBase
{
  public:
    ScoutBase() : MobileBase(){};
    ~ScoutBase() = default;

  public:
    // motion control
    void SetMotionCommand(ScoutMotionCmd<double> &cmd_);

    // get robot state
    ScoutState GetScoutState();

  private:
    // serial port buffer
    uint8_t tx_cmd_len_;
    uint8_t tx_buffer_[SCOUT_CMD_BUF_LEN];

    // cmd/status update related variables
    std::mutex scout_state_mutex_;
    std::mutex motion_cmd_mutex_;

    ScoutState scout_state_;
    ScoutMotionCmd<uint16_t> current_motion_cmd_;

    // internal functions
    void SendRobotCmd() override;
    void ParseUARTBuffer(uint8_t *buf, const size_t bufsize, size_t bytes_received);

    void SendMotionCmd();
    void NewStatusMsgReceivedCallback(const ScoutMessage &msg);

  public:
    static void UpdateScoutState(const ScoutMessage &status_msg, ScoutState &state);
};
} // namespace westonrobot

#endif /* SCOUT_BASE_HPP */
