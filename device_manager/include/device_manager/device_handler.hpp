// Copyright (c) 2023 Beijing Xiaomi Mobile Software Co., Ltd. All rights reserved.
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
#ifndef DEVICE_MANAGER__DEVICE_HANDLER_HPP_
#define DEVICE_MANAGER__DEVICE_HANDLER_HPP_

#include <string>
#include <memory>
#include <thread>
#include <vector>
#include <map>
#include <functional>
#include <condition_variable>

#include "std_msgs/msg/int32.hpp"
#include "std_msgs/msg/bool.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "device_manager/device_config.hpp"
#include "cyberdog_common/cyberdog_log.hpp"
#include "device_manager/self_check.hpp"

namespace cyberdog
{
namespace device
{
class DeviceHandler
{
  LOGGER_MINOR_INSTANCE("device_handler");

public:
  DeviceHandler() {}
  ~DeviceHandler() {}

  void Config();
  bool Init(rclcpp::Node::SharedPtr node_ptr);
  int32_t SelfCheck();

public:
  void ExecuteLed(
    const protocol::srv::LedExecute_Request::SharedPtr request,
    protocol::srv::LedExecute_Response::SharedPtr response);

  void ExecuteBmsControl(
    const protocol::srv::BmsCmd_Request::SharedPtr request,
    protocol::srv::BmsCmd_Response::SharedPtr response);

  void ExecuteUwb(
    const protocol::srv::GetUWBMacSessionID_Request::SharedPtr request,
    protocol::srv::GetUWBMacSessionID_Response::SharedPtr response);

  void UwbConnectionSignal(const std_msgs::msg::Bool::SharedPtr msg);

  void PublishTouch(protocol::msg::TouchStatus msg);
  void PublishBmsMessage(protocol::msg::BmsStatus msg);
  void PublishUwbMessage(protocol::msg::UwbRaw msg);

private:
  std::vector<std::string> simulator_;
  std::vector<std::string> device_vec_;
  std::map<std::string, std::string> device_map_;

private:
  std::shared_ptr<LedBase> led_ptr {nullptr};
  std::shared_ptr<TouchBase> touch_ptr {nullptr};
  std::shared_ptr<BMSBase> bms_ptr_ {nullptr};
  std::shared_ptr<UWBBase> uwb_ptr_ {nullptr};
  std::unique_ptr<DeviceSelfCheck> device_self_check_ptr_ {nullptr};

  rclcpp::Publisher<protocol::msg::TouchStatus>::SharedPtr touch_pub_ {nullptr};
  rclcpp::Publisher<protocol::msg::BmsStatus>::SharedPtr bms_pub_ {nullptr};
  rclcpp::Publisher<protocol::msg::UwbRaw>::SharedPtr uwb_pub_ {nullptr};
  bool led_inited_ {false};
  bool touch_inited_ {false};
  bool bms_inited_ {false};
  bool uwb_inited_ {false};
};  // class DeviceHandler

}  // namespace device
}  // namespace cyberdog

#endif  // DEVICE_MANAGER__DEVICE_HANDLER_HPP_
