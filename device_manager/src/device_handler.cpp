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
#include <vector>
#include "pluginlib/class_loader.hpp"
#include "device_manager/device_handler.hpp"
#include "cyberdog_touch/touch_plugin.hpp"
#include "cyberdog_bms/bms_plugin.hpp"
#include "cyberdog_uwb/uwb_plugin.hpp"

void cyberdog::device::DeviceHandler::Config()
{
  cyberdog::device::GetDeviceNames(device_map_);
}

bool cyberdog::device::DeviceHandler::Init(rclcpp::Node::SharedPtr node_ptr)
{
  // std::for_each(device_map_.begin(), device_map_.end(),
  //   [this](std::map<std::string, std::string>::reference name) {
  //     pluginlib::ClassLoader<cyberdog::device::CyberDogLed> led_loader(name.first, name.second);
  //     this->
  //   });
  pluginlib::ClassLoader<cyberdog::device::LedBase> led_loader("cyberdog_led",
    "cyberdog::device::LedBase");

  pluginlib::ClassLoader<cyberdog::device::BMSBase> bms_loader("cyberdog_bms",
    "cyberdog::device::BMSBase");

  pluginlib::ClassLoader<cyberdog::device::TouchBase> touch_loader("cyberdog_touch",
    "cyberdog::device::TouchBase");

  pluginlib::ClassLoader<cyberdog::device::UWBBase> uwb_loader("cyberdog_uwb",
    "cyberdog::device::UWBBase");

  led_ptr = led_loader.createSharedInstance("cyberdog::device::LedCarpo");
  touch_ptr = touch_loader.createSharedInstance("cyberdog::device::TouchCarpo");
  bms_ptr_ = bms_loader.createSharedInstance("cyberdog::device::BMSCarpo");
  uwb_ptr_ = uwb_loader.createSharedInstance("cyberdog::device::UWBCarpo");

  touch_pub_ = node_ptr->create_publisher<protocol::msg::TouchStatus>("touch_status", 10);
  bms_pub_ = node_ptr->create_publisher<protocol::msg::BmsStatus>("bms_status", 10);
  uwb_pub_ = node_ptr->create_publisher<protocol::msg::UwbArray>("uwb_raw", 10);

  node_ptr->declare_parameter("simulator", std::vector<std::string>{});
  node_ptr->get_parameter("simulator", this->simulator_);
  auto is_simulator = [this](std::string sensor_name) -> bool {
      return static_cast<bool>(std::find(
               this->simulator_.begin(), this->simulator_.end(),
               sensor_name) != this->simulator_.end());
    };

  if (!led_ptr->Init()) {
    ERROR("Led int fail.");
    return false;
  }
  if (!touch_ptr->Init(
      std::bind(&DeviceHandler::PublishTouch, this, std::placeholders::_1),
      is_simulator("touch")))
  {
    ERROR("Touch int fail.");
    return false;
  }
  if (!bms_ptr_->Init(
      std::bind(&DeviceHandler::PublishBmsMessage, this, std::placeholders::_1),
      is_simulator("bms")))
  {
    ERROR("Bms int fail.");
    return false;
  }
  if (!uwb_ptr_->Init(
      std::bind(&DeviceHandler::PublishUwbMessage, this, std::placeholders::_1),
      is_simulator("uwb")))
  {
    ERROR("Uwb int fail.");
    return false;
  }
  return true;
}

bool cyberdog::device::DeviceHandler::SelfCheck()
{
  INFO("DeviceManager SelfCheck begin");
  if (!led_ptr->SelfCheck()) {
    ERROR("Led selfcheck fail.");
    return false;
  }
  if (!touch_ptr->SelfCheck()) {
    ERROR("Touch selfcheck fail.");
    return false;
  }
  if (!bms_ptr_->SelfCheck()) {
    ERROR("Bms selfcheck fail.");
    return false;
  }
  if (!uwb_ptr_->SelfCheck()) {
    ERROR("Uwb selfcheck fail.");
    return false;
  }
  return true;
}

void cyberdog::device::DeviceHandler::ExecuteLed(
  const protocol::srv::LedExecute_Request::SharedPtr request,
  protocol::srv::LedExecute_Response::SharedPtr response)
{
  led_ptr->Play(request, response);
}

void cyberdog::device::DeviceHandler::ExecuteBmsControl(
  const protocol::srv::BmsCmd_Request::SharedPtr request,
  protocol::srv::BmsCmd_Response::SharedPtr response)
{
  bms_ptr_->ServiceCommand(request, response);
}

void cyberdog::device::DeviceHandler::ExecuteUwb(
  const protocol::srv::GetUWBMacSessionID_Request::SharedPtr request,
  protocol::srv::GetUWBMacSessionID_Response::SharedPtr response)
{
  uwb_ptr_->Play(request, response);
}

void cyberdog::device::DeviceHandler::UwbConnectionSignal(
  const std_msgs::msg::Bool::SharedPtr msg)
{
  uwb_ptr_->SetConnectedState(msg->data);
}

void cyberdog::device::DeviceHandler::PublishTouch(protocol::msg::TouchStatus msg)
{
  if (touch_pub_ != nullptr) {
    touch_pub_->publish(msg);
  }
}

void cyberdog::device::DeviceHandler::PublishBmsMessage(protocol::msg::BmsStatus msg)
{
  if (bms_pub_ != nullptr) {
    bms_pub_->publish(msg);
  }
}

void cyberdog::device::DeviceHandler::PublishUwbMessage(protocol::msg::UwbArray msg)
{
  if (uwb_pub_ != nullptr) {
    uwb_pub_->publish(msg);
  }
}
