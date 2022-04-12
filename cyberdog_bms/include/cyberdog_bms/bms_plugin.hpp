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

#include "cyberdog_bms/bms_base.hpp"
#include "cyberdog_common/cyberdog_log.hpp"
#include "rclcpp/rclcpp.hpp"
#include "embed_protocol/embed_protocol.hpp"
namespace cyberdog
{
namespace device
{

struct BMSStatus
{
  // 电量	电压	电流	温度	循环次数	健康度	故障状态
  // battery_status[0]   : 电量
  // battery_status[1]   : 电压
  // battery_status[2]   : 电流
  // battery_status[3]   : 温度
  // battery_status[4-5] : 循环次数
  // battery_status[6]   : 健康度
  // battery_status[7]   : 故障状态
  std::array<uint8_t, 8> battery_status;

  // 01测试通过	01电量正常	01SC8815正常	01CYPD3171正常	01CAN正常	01串口正常
  // 01测试失败	01电量异常	01SC8815异常	01CYPD3171异常	01CAN异常	01串口异常
  std::array<uint8_t, 6> abnormal_status;
  std::array<uint8_t, 6> normal_status;

  // 0x01(正常模式)
  // 0x02(正在充电)
  // 0x03(充电完成)
  // 0x04(电机掉电)
  // 0x05(软关机)
  uint8_t normal_mode;
  uint8_t charging;
  uint8_t finished_charging;
  uint8_t motor_power_down;
  uint8_t soft_shutdown;

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

  // uint8_t test_failed;
  // uint8_t test_passed;

  // // 电量
  // uint8_t battery_abnormal;
  // uint8_t battery_normal;

  // // SC8815
  // uint8_t SC8815_abnormal;
  // uint8_t SC8815_normal;

  // // CYPD3171
  // uint8_t CYPD3171_abnormal;
  // uint8_t CYPD3171_normal;

  // // CAN
  // uint8_t CAN_abnormal;
  // uint8_t CAN_normal;

  // // 串口
  // uint8_t serial_port_abnormal;
  // uint8_t serial_port_normal;
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

class BMSCarpo : public cyberdog::device::BMSBase
{
public:
  using BmsStatusMsg = protocol::msg::Bms;
  using BMSCanPtr = std::shared_ptr<cyberdog::embed::Protocol<BMSStatus>>;

  BMSCarpo();
  virtual bool Config();
  virtual bool Init(std::function<void(BmsStatusMsg)> function_callback, bool simulation = false);
  virtual bool SelfCheck();
  virtual bool RegisterTopic(std::function<void(BmsStatusMsg)> function_callback);
  virtual void Report(
    const std::shared_ptr<protocol::srv::BmsInfo::Request> request,
    std::shared_ptr<protocol::srv::BmsInfo::Response> response);

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
  void HandleBMSMessages(std::string & name, std::shared_ptr<BMSStatus> data);

  // command status for other device
  bool SendCommand(const Command & command);

  // Set battery_status
  void SetBatteryStatusData(const std::array<uint8_t, 8>& data);

  // Set abnormal_status
  void SetAbnormalStatusData(const std::array<uint8_t, 6>& data);

  // Set normal_status
  void SetNormalStatusData(const std::array<uint8_t, 6>& data);

  // Set normal_mode
  void SetNormalModeData(uint8_t normal_mode);

  // Set charging
  void SetChargingData(uint8_t charging);

  // Set finished_charging
  void SetFinishedChargingData(uint8_t finished_charging);

  // Set motor_power_down
  void SetMotorPowerDownData(uint8_t motor_power_down);

  // Set soft_shutdown
  void SetSoftShutdownData(uint8_t soft_shutdown);

  // Convert BMSStatus to protocol::msg::Bms
  protocol::msg::Bms ToRos(const BMSStatus & can_data);
  void DebugString();

  // Dimulation Data for debug
  void RunSimulation(); 

  // Generate random number
  int GenerateRandomNumber(int start, int end);

  protocol::msg::Bms bms_message_;
  BMSStatus can_message_;

  std::function<void(BmsStatusMsg)> status_function_;
  std::thread bms_thread_;
  std::thread bms_test_thread_;
  bool initialized_finished_ {false};
  bool simulation_ {false};
  bool test_ {false};
  int test_command_;
  std::mutex test_mutex_;
  BMSCanPtr can_bridge_ {nullptr};
};  //  class BMSCarpo
}   //  namespace device
}   //  namespace cyberdog

#endif  // CYBERDOG_BMS__BMS_PLUGIN_HPP_
