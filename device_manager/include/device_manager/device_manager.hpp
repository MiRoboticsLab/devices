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
#ifndef DEVICE_MANAGER__DEVICE_MANAGER_HPP_
#define DEVICE_MANAGER__DEVICE_MANAGER_HPP_

#include <string>
#include <memory>

#include "std_msgs/msg/bool.hpp"
#include "manager_base/manager_base.hpp"
#include "device_manager/device_handler.hpp"
#include "cyberdog_machine/cyberdog_fs_machine.hpp"
#include "cyberdog_machine/cyberdog_heartbeats.hpp"

namespace cyberdog
{
namespace device
{
enum class DeviceErrorCode : int32_t
{
  kDemoError1 = 21,
  kDemoError2 = 22,
  kDemoError3 = 23
};

class DeviceManager : public cyberdog::machine::MachineActuator
{
public:
  explicit DeviceManager(const std::string & name);
  ~DeviceManager();

  void Config();
  bool Init();
  void Run();
  int32_t SelfCheck();

public:
  int32_t OnError();
  int32_t OnLowPower();
  int32_t OnSuspend();
  int32_t OnProtected();
  int32_t OnActive();
  int32_t OnDeActive();
  int32_t OnSetUp();
  int32_t ONTearDown();
  int32_t OnOTA();

private:
  bool IsStateInvalid();

private:
  void LedServiceCallback(
    const protocol::srv::LedExecute_Request::SharedPtr request,
    protocol::srv::LedExecute_Response::SharedPtr response);

  void BmsControlCallback(
    const protocol::srv::BmsCmd_Request::SharedPtr request,
    protocol::srv::BmsCmd_Response::SharedPtr response);

  void UwbServiceCallback(
    const protocol::srv::GetUWBMacSessionID_Request::SharedPtr request,
    protocol::srv::GetUWBMacSessionID_Response::SharedPtr response);

  void UwbConnectedCallback(const std_msgs::msg::Bool::ConstPtr msg);

private:
  std::string name_;
  std::shared_ptr<DeviceHandler> device_handler_ {nullptr};
  rclcpp::Node::SharedPtr node_ptr {nullptr};
  rclcpp::Service<protocol::srv::LedExecute>::SharedPtr led_service_ {nullptr};
  rclcpp::Service<protocol::srv::BmsCmd>::SharedPtr bms_service_ {nullptr};
  rclcpp::Service<protocol::srv::GetUWBMacSessionID>::SharedPtr uwb_service_ {nullptr};
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr uwb_connection_state_ {nullptr};
  std::unique_ptr<cyberdog::machine::HeartBeatsActuator> heart_beats_ptr_ {nullptr};
  std::shared_ptr<cyberdog::system::CyberdogCode<DeviceErrorCode>> code_ptr_ {nullptr};
  rclcpp::executors::MultiThreadedExecutor executor;
  bool is_active {false};

private:
  const std::string Uninitialized_V = std::string("Uninitialized");
  const std::string SetUp_V = std::string("SetUp");
  const std::string TearDown_V = std::string("TearDown");
  const std::string SelfCheck_V = std::string("SelfCheck");
  const std::string Active_V = std::string("Active");
  const std::string DeActive_V = std::string("DeActive");
  const std::string Protected_V = std::string("Protected");
  const std::string LowPower_V = std::string("LowPower");
  const std::string OTA_V = std::string("OTA");
  const std::string Error_V = std::string("Error");
};  // class DeviceManager
}  // namespace device
}  // namespace cyberdog

#endif  //  DEVICE_MANAGER__DEVICE_MANAGER_HPP_
