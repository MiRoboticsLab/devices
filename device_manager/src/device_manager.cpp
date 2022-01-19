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
#include "rclcpp/rclcpp.hpp"
#include "device_manager/device_manager.hpp"

cyberdog::device::DeviceManager::DeviceManager(const std::string& name)
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
  // TODO: config priority and robot states
}

bool cyberdog::device::DeviceManager::Init()
{
  if(!device_handler_->Init(node_ptr)) {
    // error msg
    return false;
  }
  if(!RegisterStateHandler(node_ptr)) {
    return false;
  } else {
    if(! RegisterInitHandler(node_ptr)) {
      return false;
    } else {
      if(!RegisterHeartbeats(node_ptr)) {
        return false;
      }
    }
  }

  led_service_ = node_ptr->create_service<protocol::srv::LedExecute>("led_execute",
    std::bind(&DeviceManager::LedServiceCallback, this,
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
  std::cout << "on error\n";
}

void cyberdog::device::DeviceManager::OnLowPower()
{
  std::cout << "on lowpower\n";
}

void cyberdog::device::DeviceManager::OnSuspend()
{
  std::cout << "on suspend\n";
}

void cyberdog::device::DeviceManager::OnProtected()
{
  std::cout << "on protect\n";
}

void cyberdog::device::DeviceManager::OnActive()
{
  std::cout << "on active\n";
}

bool cyberdog::device::DeviceManager::IsStateInvalid()
{
  // TODO: check from config priority and current states
  return true;
}

void cyberdog::device::DeviceManager::LedServiceCallback(const protocol::srv::LedExecute_Request::SharedPtr request,
    protocol::srv::LedExecute_Response::SharedPtr response)
{
  std::cout << "led service~\n";
  if(!IsStateInvalid()){
    response->code = (int32_t)system::KeyCode::kStateInvalid;
    return;
  }
  std::cout << "led service will into handler~\n";
  device_handler_->ExecuteLed(request, response);
}