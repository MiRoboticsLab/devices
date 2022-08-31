// Copyright (c) 2021 Beijing Xiaomi Mobile Software Co., Ltd. All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef CYBERDOG_BMS__BMS_PLUGIN_HPP_
#define CYBERDOG_BMS__BMS_PLUGIN_HPP_

#include <array>
#include <memory>
#include <string>
#include <thread>
#include <chrono>
#include <random>
#include <mutex>
#include <deque>

#include "cyberdog_bms/bms_base.hpp"
#include "cyberdog_common/cyberdog_log.hpp"
#include "rclcpp/rclcpp.hpp"
#include "embed_protocol/embed_protocol.hpp"
#include "sensor_msgs/msg/battery_state.hpp"

namespace cyberdog
{
namespace device
{

struct BatteryStatus
{
  std::array<uint8_t, 16> battery_status;
  // 00 : normal
  // 01 : abnormal
  std::array<uint8_t, 6> normal_status;

  // 0x01(正常模式）
  // 0x02(关闭电机）
  // 0x03(低功耗）
  // 0x04(软关机）
  uint8_t cmd_normal_mode;
  uint8_t cmd_turn_off_motor;
  uint8_t cmd_low_power_consumption;
  uint8_t cmd_soft_shutdown;

  // 文件传输
  // OTA升级
  uint16_t cmd_file_transfer;
  uint16_t cmd_ota_upgrade;

  // 测试
  uint8_t cmd_test;
};


enum class Command
{
  // 0x01(正常模式）
  // 0x02(关闭电机）
  // 0x03(低功耗）
  // 0x04(软关机）
  kNormalMode,
  kTurnOffMotor,
  kLowPowerConsumption,
  kSoftShutdown,

  // 文件传输
  // OTA升级
  kFileTransfer,
  kOTAUpgrade,

  // 0x01（测试）
  kTest
};


enum class PrintMessageType
{
  kBatteryStatus,
  kBatteryTestNormalStatus,
};

class BMSCarpo : public cyberdog::device::BMSBase
{
public:
  using BmsStatusMsg = protocol::msg::BmsStatus;
  using BatterySharedPtr = std::shared_ptr<cyberdog::embed::Protocol<BatteryStatus>>;

  BMSCarpo();
  virtual bool Config();
  virtual bool Init(std::function<void(BmsStatusMsg)> function_callback, bool simulation = false);
  virtual bool SelfCheck();
  virtual bool RegisterTopic(std::function<void(BmsStatusMsg)> function_callback);
  virtual void ServiceCommand(
    const std::shared_ptr<protocol::srv::BmsCmd::Request> request,
    std::shared_ptr<protocol::srv::BmsCmd::Response> response);

  // Test
  void RunTest();

  // Stop command.
  void StopTest();

  // Start command.
  void StartTest();

  // Set which command case run.
  void SetTestCase(int test_case);

private:
  // Handle bms task and send process data
  void RunBmsTask();

  //  Initialize bms message pointer
  void InitializeBmsProtocol();

  // Get protocol go though by emv
  void HandleBatteryStatusMessages(std::string & name, std::shared_ptr<BatteryStatus> data);

  // command status for other device
  bool SendCommand(const Command & command);

  // battery state
  void SetBatteryStatus(const std::array<uint8_t, 16> & data);

  // normal data
  void SetNormalStatus(std::array<uint8_t, 6> data);

  // Convert BatteryStatus to protocol::msg::Bms
  protocol::msg::BmsStatus ToRos(const BatteryStatus & can_data);
  void DebugString(const PrintMessageType & type);

  // Dimulation Data for debug
  void RunSimulation();

  // Generate random number
  int GenerateRandomNumber(int start, int end);

  protocol::msg::BmsStatus ros_bms_message_;
  BatteryStatus can_battery_message_;

  std::function<void(BmsStatusMsg)> status_function_;
  std::thread bms_thread_;
  std::thread bms_test_thread_;
  bool initialized_finished_ {false};
  bool simulation_ {false};
  bool test_ {false};
  int test_command_;
  std::mutex test_mutex_;
  std::mutex mutex_battery_;
  BatterySharedPtr battery_status_ptr_ {nullptr};

  // http://docs.ros.org/en/noetic/api/sensor_msgs/html/msg/BatteryState.html
  // ROS2 interface
  sensor_msgs::msg::BatteryState battery_state_;
  std::deque<protocol::msg::BmsStatus> queue_;
};  //  class BMSCarpo
}   //  namespace device
}   //  namespace cyberdog

#endif  // CYBERDOG_BMS__BMS_PLUGIN_HPP_
