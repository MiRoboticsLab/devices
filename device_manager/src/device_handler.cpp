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
#include "cyberdog_touch/touch_plugin.hpp"
#include "cyberdog_bms/bms_plugin.hpp"

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
  led_ptr = led_loader.createSharedInstance("cyberdog::device::LedCarpo");
  led_ptr->Init();

  pluginlib::ClassLoader<cyberdog::device::BMSBase> bms_loader("cyberdog_bms",
    "cyberdog::device::BMSBase");
  bms_ptr_ = bms_loader.createSharedInstance("cyberdog::device::BMSCarpo");
  // touch_pub_ = node_ptr->create_publisher<protocol::msg::TouchStatus>("touch_status", 10);
  // touch_ptr->Init(std::bind(&DeviceHandler::PublishTouch, this, std::placeholders::_1), true);

  bms_pub_ = node_ptr->create_publisher<protocol::msg::Bms>("bms_status", 10);
  bms_ptr_->Init(std::bind(&DeviceHandler::PublishBmsMessage, this, std::placeholders::_1));

  bms_test_subscription_ = node_ptr->create_subscription<std_msgs::msg::Int32>(
    "bms_test", 10,
    std::bind(&DeviceHandler::HandleTestBMSCaseCallback, this, std::placeholders::_1));

  return true;
}

bool cyberdog::device::DeviceHandler::SelfCheck()
{
  return led_ptr->SelfCheck();
}

void cyberdog::device::DeviceHandler::ExecuteLed(
  const protocol::srv::LedExecute_Request::SharedPtr request,
  protocol::srv::LedExecute_Response::SharedPtr response)
{
  led_ptr->Play(request, response);
}

void cyberdog::device::DeviceHandler::PublishTouch(protocol::msg::TouchStatus msg)
{
  // if(touch_pub_ != nullptr) {
  //   touch_pub_->publish(msg);
  // }
}

void cyberdog::device::DeviceHandler::PublishBmsMessage(protocol::msg::Bms msg)
{
  if (bms_pub_ != nullptr) {
    bms_pub_->publish(msg);
  }
}


#if 0
void cyberdog::device::DeviceHandler::HandleTestBMSCaseCallback(const std_msgs::msg::Int32 & msg)
{
  if (msg.data >= 10) {
    dynamic_cast<BMSCarpo *>(bms_ptr_.get())->StopTest();
  }
  dynamic_cast<BMSCarpo *>(bms_ptr_.get())->SetTestCase(msg.data);
  dynamic_cast<BMSCarpo *>(bms_ptr_.get())->RunTest();
}
#else
void cyberdog::device::DeviceHandler::HandleTestBMSCaseCallback(
  const std_msgs::msg::Int32::SharedPtr msg)
{
  if (msg->data >= 10) {
    dynamic_cast<BMSCarpo *>(bms_ptr_.get())->StopTest();
  }
  dynamic_cast<BMSCarpo *>(bms_ptr_.get())->SetTestCase(msg->data);
  dynamic_cast<BMSCarpo *>(bms_ptr_.get())->RunTest();
}
#endif
