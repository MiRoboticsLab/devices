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

#ifndef CYBERDOG_BLUETOOTH__BLUETOOTH_BASE_HPP_
#define CYBERDOG_BLUETOOTH__BLUETOOTH_BASE_HPP_


#include <string>
#include <memory>
#include "rclcpp/rclcpp.hpp"


namespace cyberdog
{
namespace device
{

class BluetoothBase
{
public:
  // using BmsStatusMsg = protocol::msg::BmsStatus;

  virtual bool Config() = 0;
  // virtual bool Init(std::function<void(BmsStatusMsg)>
  // function_callback, bool simulation = false) = 0;
  virtual bool SelfCheck() = 0;
  // virtual bool RegisterTopic(std::function<void(BmsStatusMsg)> function_callback) = 0;

protected:
  BluetoothBase() {}
};  // class BluetoothBase
}  // namespace device
}  // namespace cyberdog

#endif  // CYBERDOG_BLUETOOTH__BLUETOOTH_BASE_HPP_
