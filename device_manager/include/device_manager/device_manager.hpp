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

#include "manager_base/manager_base.hpp"
#include "device_manager/device_handler.hpp"

namespace cyberdog
{
namespace device
{
class DeviceManager : public manager::ManagerBase
{
public:
  explicit DeviceManager(const std::string & name);
  ~DeviceManager();

  void Config() override;
  bool Init() override;
  void Run() override;
  bool SelfCheck() override;

public:
  void OnError() override;
  void OnLowPower() override;
  void OnSuspend() override;
  void OnProtected() override;
  void OnActive() override;

private:
  bool IsStateInvalid();

private:
  void LedServiceCallback(
    const protocol::srv::LedExecute_Request::SharedPtr request,
    protocol::srv::LedExecute_Response::SharedPtr response);

  void BmsControlCallback(
    const protocol::srv::BmsCmd_Request::SharedPtr request,
    protocol::srv::BmsCmd_Response::SharedPtr response);

private:
  std::string name_;
  std::shared_ptr<DeviceHandler> device_handler_ {nullptr};
  rclcpp::Node::SharedPtr node_ptr {nullptr};
  rclcpp::Service<protocol::srv::LedExecute>::SharedPtr led_service_ {nullptr};
  rclcpp::Service<protocol::srv::BmsCmd>::SharedPtr bms_service_ {nullptr};
};  // class DeviceManager
}  // namespace device
}  // namespace cyberdog

#endif  //  DEVICE_MANAGER__DEVICE_MANAGER_HPP_
