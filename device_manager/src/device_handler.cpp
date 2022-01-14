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
#include "pluginlib/class_loader.hpp"
#include "device_manager/device_handler.hpp"

void cyberdog::device::DeviceHandler::Config()
{
  cyberdog::device::GetDeviceNames(device_map_);
}

bool cyberdog::device::DeviceHandler::Init()
{
  // std::for_each(device_map_.begin(), device_map_.end(),
  //   [this](std::map<std::string, std::string>::reference name) {
  //     pluginlib::ClassLoader<cyberdog::device::CyberDogLed> led_loader(name.first, name.second);
  //     this->
  //   });

  pluginlib::ClassLoader<cyberdog::device::LedBase> led_loader("cyberdog_led", "LedBase");
  led_ptr = led_loader.createSharedInstance("LedCarpo");
  // node_ptr->create_service<>
  return true;
}

bool cyberdog::device::DeviceHandler::SelfCheck()
{
  return led_ptr->SelfCheck();
}

void cyberdog::device::DeviceHandler::ExecuteLed(const protocol::srv::LedExecute_Request request,
    protocol::srv::LedExecute_Response& response)
{
  led_ptr->Play(request, response);
}

