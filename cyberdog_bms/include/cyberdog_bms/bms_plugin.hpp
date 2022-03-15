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

#include <memory>
#include <string>
#include <thread>
#include <chrono>

#include "cyberdog_bms/bms_base.hpp"
#include "cyberdog_common/cyberdog_log.hpp"
#include "rclcpp/rclcpp.hpp"
#include "embed_protocol/embed_protocol.hpp"
namespace cyberdog
{
namespace device
{

typedef struct CanProtocolBmsType
{
  uint16_t batt_volt;
  int16_t batt_curr;
  uint8_t batt_soc;
  int16_t batt_temp;
  uint8_t batt_st;
  uint8_t key_val;
  uint8_t disable_charge;
  uint8_t power_supply;
  uint8_t buzze;
  uint8_t status;
  int8_t batt_health;
  int16_t batt_loop_number;
  int8_t powerboard_status;
} CanProtocolBmsType;

enum Command
{
  kBuzze,
  kPowerSupply,
  kDisableCharge
};

class BMSCarpo : public cyberdog::device::BMSBase
{
public:
  using BmsStatusMsg = protocol::msg::Bms;
  using BMSCanPtr = std::shared_ptr<cyberdog::embed::Protocol<CanProtocolBmsType>>;

  BMSCarpo();
  virtual bool Config();
  virtual bool Init(std::function<void(BmsStatusMsg)> function_callback);
  virtual bool SelfCheck();
  virtual bool RegisterTopic(std::function<void(BmsStatusMsg)> function_callback);
  virtual void Report(
    const std::shared_ptr<protocol::srv::BmsInfo::Request> request,
    std::shared_ptr<protocol::srv::BmsInfo::Response> response);

private:
  void RunBmsTask();
  //  Initialize bms message pointer
  void InitializeBmsProtocol();
  // Get protocol go though by emv
  void HandleBMSMessages(std::string & name, std::shared_ptr<CanProtocolBmsType> data);
  // command status for other device
  bool SendCommand(const Command & command);
  // Convert CanProtocolBmsType to protocol::msg::Bms
  protocol::msg::Bms ToRos(const CanProtocolBmsType & can_data);
  void DebugString();

  protocol::msg::Bms bms_message_;
  std::function<void(BmsStatusMsg)> status_function_;
  std::thread bms_thread_;
  bool initialized_finished_ {false};
  BMSCanPtr bms_can_bridge_ {nullptr};
};  //  class BMSCarpo
}   //  namespace device
}   //  namespace cyberdog

#endif  // CYBERDOG_BMS__BMS_PLUGIN_HPP_
