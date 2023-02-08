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

#ifndef CYBERDOG_UWB__UWB_BASE_HPP_
#define CYBERDOG_UWB__UWB_BASE_HPP_

#include <string>
#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

#include "protocol/msg/uwb_raw.hpp"
#include "protocol/msg/uwb_array.hpp"
#include "protocol/srv/get_uwb_mac_session_id.hpp"

namespace cyberdog
{
namespace device
{

class UWBBase
{
public:
  using UwbRawStatusMsg = protocol::msg::UwbArray;
  using UwbSignleStatusMsg = protocol::msg::UwbRaw;
  virtual ~UWBBase() {}
  virtual bool Config() = 0;
  virtual bool Init(
    std::function<void(UwbSignleStatusMsg)>
    function_callback, bool simulation = false) = 0;
  virtual int32_t SelfCheck() = 0;
  virtual bool LowPower() = 0;
  virtual bool RegisterTopic(
    std::function<void(UwbSignleStatusMsg)> function_callback) = 0;
  virtual void Play(
    const std::shared_ptr<protocol::srv::GetUWBMacSessionID::Request> info_request,
    std::shared_ptr<protocol::srv::GetUWBMacSessionID::Response> info_response) = 0;
  virtual void SetConnectedState(bool connected) = 0;

protected:
  UWBBase() {}
};  // class UWBBase
}  // namespace device
}  // namespace cyberdog

#endif  // CYBERDOG_UWB__UWB_BASE_HPP_
