// Copyright (c) 2023-2023 Beijing Xiaomi Mobile Software Co., Ltd. All rights reserved.
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
#include "cyberdog_common/cyberdog_semaphore.hpp"
#include "cyberdog_system/robot_code.hpp"
#include "rclcpp/rclcpp.hpp"
#include "embed_protocol/embed_protocol.hpp"
#include "sensor_msgs/msg/battery_state.hpp"

namespace cyberdog
{
namespace device
{
namespace EP = cyberdog::embed;
namespace SYS = cyberdog::system;

class BMSCarpo : public cyberdog::device::BMSBase
{
public:
  using BmsStatusMsg = protocol::msg::BmsStatus;

  BMSCarpo();
  virtual bool Config();
  virtual bool Init(std::function<void(BmsStatusMsg)> function_callback, bool simulation = false);
  virtual int32_t SelfCheck();
  virtual bool RegisterTopic(std::function<void(BmsStatusMsg)> publisher);
  virtual void ServiceCommand(
    const std::shared_ptr<protocol::srv::BmsCmd::Request> request,
    std::shared_ptr<protocol::srv::BmsCmd::Response> response);
  bool LowPower() override;
  // Open and Close can
  bool Open();
  bool Close();

public:
  typedef struct
  {
    union {
      uint8_t battery_status[16];
      struct
      {
        uint8_t batt_soc;
        uint16_t batt_volt;
        uint16_t batt_curr;
        uint8_t batt_temp;
        uint8_t power_adapter_temp;
        uint8_t wireless_charging_temp;
        uint16_t batt_loop_number;
        uint8_t batt_health;
        uint16_t batt_st;
        union {
          uint8_t bms_state1;
          struct
          {
            uint8_t power_normal : 1;
            uint8_t power_wired_charging : 1;
            uint8_t power_finished_charging : 1;
            uint8_t power_motor_shutdown : 1;
            uint8_t power_soft_shutdown : 1;
            uint8_t power_wp_place : 1;
            uint8_t power_wp_charging : 1;
            uint8_t power_expower_supply : 1;
          };
        };
        union {
          uint8_t bms_state2;
          struct
          {
            uint8_t power_off_charging : 1;
            uint8_t bms_state2_reserved : 7;
          };
        };
        uint8_t reserved;
      } __attribute__((packed));
    };
    // ack
    uint8_t bms_enable_on_ack;
    uint8_t bms_enable_off_ack;
    cyberdog::common::Semaphore enable_on_signal;
    cyberdog::common::Semaphore enable_off_signal;
    cyberdog::common::Semaphore battery_status_signal;
    std::atomic<bool> data_received;
    std::atomic<bool> waiting_data;
    std::chrono::system_clock::time_point time_start;
  } BatteryMsg;
  enum class Command
  {
    // 0x01(正常模式）
    // 0x02(关闭电机)
    // 0x03(开启无线充电)
    // 0x04(关闭无线充电)
    kNormalMode,
    kTurnOffMotor,
    kTurnOnWirelessCharging,
    kTurnOffWirelessCharging
  };
  enum class BMS_Code : int32_t
  {
    kDemoError1 = 21
  };

private:
  bool simulator_ {false};
  std::shared_ptr<EP::Protocol<BatteryMsg>> battery_ {nullptr};
  std::atomic<bool> is_working_ = false;
  std::function<void(BmsStatusMsg)> topic_pub_ = nullptr;
  std::thread simulator_thread_;
  std::shared_ptr<SYS::CyberdogCode<BMS_Code>> code_{nullptr};

private:
  void SimulationThread();
  //  Initialize bms message pointer
  void InitializeBmsProtocol();
  // Get protocol go though by emv
  void BatteryMsgCall(EP::DataLabel & label, std::shared_ptr<BatteryMsg> data);
  // command status for other device
  bool SendCommand(const Command & command);
  // Generate random number
  int GenerateRandomNumber(int start, int end);
};  //  class BMSCarpo
}   //  namespace device
}   //  namespace cyberdog

#endif  // CYBERDOG_BMS__BMS_PLUGIN_HPP_
