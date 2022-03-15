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

#ifndef CYBERDOG_BMS__BMS_BASE_HPP_
#define CYBERDOG_BMS__BMS_BASE_HPP_

#include <string>
#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "protocol/srv/led_execute.hpp"
#include "protocol/msg/bms.hpp"
#include "protocol/srv/bms_info.hpp"

namespace cyberdog
{
namespace device
{

class BMSBase
{
public:
  using BmsStatusMsg = protocol::msg::Bms;

  virtual bool Config() = 0;
  virtual bool Init(std::function<void(BmsStatusMsg)> function_callback) = 0;
  virtual bool SelfCheck() = 0;
  virtual bool RegisterTopic(std::function<void(BmsStatusMsg)> function_callback) = 0;
  virtual void Report(
    const std::shared_ptr<protocol::srv::BmsInfo::Request> request,
    std::shared_ptr<protocol::srv::BmsInfo::Response> response) = 0;

protected:
  BMSBase() {}
};  // class BMSBase
}  // namespace device
}  // namespace cyberdog

#endif  // CYBERDOG_BMS__BMS_BASE_HPP_
