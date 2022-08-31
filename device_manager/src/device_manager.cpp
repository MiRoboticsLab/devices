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

#include <string>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "device_manager/device_manager.hpp"

cyberdog::device::DeviceManager::DeviceManager(const std::string & name)
: manager::ManagerBase(name),
  name_(name)
{
  node_ptr = rclcpp::Node::make_shared(name_);
  device_handler_ = std::make_shared<cyberdog::device::DeviceHandler>();
}

cyberdog::device::DeviceManager::~DeviceManager()
{}

void cyberdog::device::DeviceManager::Config()
{
  device_handler_->Config();
  // config priority and robot states
}

bool cyberdog::device::DeviceManager::Init()
{
  if (!device_handler_->Init(node_ptr)) {
    // error msg
    return false;
  }
  if (!RegisterStateHandler(node_ptr)) {
    return false;
  } else {
    if (!RegisterInitHandler(node_ptr)) {
      return false;
    } else {
      if (!RegisterHeartbeats(node_ptr)) {
        return false;
      }
    }
  }

  led_service_ = node_ptr->create_service<protocol::srv::LedExecute>(
    "led_execute",
    std::bind(
      &DeviceManager::LedServiceCallback, this,
      std::placeholders::_1, std::placeholders::_2));

  bms_service_ = node_ptr->create_service<protocol::srv::BmsCmd>(
    "bms_cmd",
    std::bind(
      &DeviceManager::BmsControlCallback, this,
      std::placeholders::_1, std::placeholders::_2));

  return true;
}

bool cyberdog::device::DeviceManager::SelfCheck()
{
  return device_handler_->SelfCheck();
}

void cyberdog::device::DeviceManager::Run()
{
  rclcpp::spin(node_ptr);
  rclcpp::shutdown();
}

void cyberdog::device::DeviceManager::OnError()
{
  ERROR("on error");
}

void cyberdog::device::DeviceManager::OnLowPower()
{
  ERROR("on lowpower");
}

void cyberdog::device::DeviceManager::OnSuspend()
{
  ERROR("on suspend");
}

void cyberdog::device::DeviceManager::OnProtected()
{
  ERROR("on protect");
}

void cyberdog::device::DeviceManager::OnActive()
{
  ERROR("on active");
}

bool cyberdog::device::DeviceManager::IsStateInvalid()
{
  // check from config priority and current states
  return true;
}

void cyberdog::device::DeviceManager::LedServiceCallback(
  const protocol::srv::LedExecute_Request::SharedPtr request,
  protocol::srv::LedExecute_Response::SharedPtr response)
{
  if (!IsStateInvalid()) {
    response->code = (int32_t)system::KeyCode::kStateInvalid;
    return;
  }
  device_handler_->ExecuteLed(request, response);
}

void cyberdog::device::DeviceManager::BmsControlCallback(
  const protocol::srv::BmsCmd_Request::SharedPtr request,
  protocol::srv::BmsCmd_Response::SharedPtr response)
{
  device_handler_->ExecuteBmsControl(request, response);
}
