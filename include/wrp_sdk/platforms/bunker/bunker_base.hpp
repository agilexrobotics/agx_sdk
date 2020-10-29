/*
 * bunker_base.hpp
 *
 * Created on: Jun 04, 2019 01:22
 * Description:
 *
 * Copyright (c) 2019 Ruixiang Du (rdu)
 */

#ifndef BUNKER_BASE_HPP
#define BUNKER_BASE_HPP

#include <string>
#include <cstdint>
#include <thread>
#include <mutex>
#include <functional>

#include "wrp_sdk/platforms/common/mobile_base.hpp"

#include "wrp_sdk/platforms/bunker/bunker_protocol.h"
#include "wrp_sdk/platforms/bunker/bunker_can_parser.h"
#include "wrp_sdk/platforms/bunker/bunker_uart_parser.h"
#include "wrp_sdk/platforms/bunker/bunker_types.hpp"

namespace westonrobot {
class BunkerBase : public MobileBase {
 public:
  BunkerBase() : MobileBase(){};
  ~BunkerBase() = default;

 public:
  // motion control
  void SetMotionCommand(double linear_vel, double angular_vel,
                        BunkerMotionCmd::FaultClearFlag fault_clr_flag =
                            BunkerMotionCmd::FaultClearFlag::NO_FAULT);

  // light control
  void SetLightCommand(BunkerLightCmd cmd);
  void DisableLightCmdControl();

  // get robot state
  BunkerState GetBunkerState();

 private:
  // serial port buffer
  uint8_t tx_cmd_len_;
  uint8_t tx_buffer_[BUNKER_CMD_BUF_LEN];

  // cmd/status update related variables
  std::thread cmd_thread_;
  std::mutex bunker_state_mutex_;
  std::mutex motion_cmd_mutex_;
  std::mutex light_cmd_mutex_;

  BunkerState bunker_state_;
  BunkerMotionCmd current_motion_cmd_;
  BunkerLightCmd current_light_cmd_;


 

  bool light_ctrl_enabled_ = false;
  bool light_ctrl_requested_ = false;

  // internal functions
  void SendRobotCmd() override;
  void ParseCANFrame(can_frame *rx_frame) override;
  void ParseUARTBuffer(uint8_t *buf, const size_t bufsize,
                       size_t bytes_received) override{};

  void SendMotionCmd(uint8_t count);
  void SendLightCmd(uint8_t count);
  void NewStatusMsgReceivedCallback(const BunkerMessage &msg);

 public:
  static void UpdateBunkerState(const BunkerMessage &status_msg,
                               BunkerState &state);
};
}  // namespace westonrobot

#endif /* BUNKER_BASE_HPP */
