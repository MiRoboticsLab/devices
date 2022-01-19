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
#ifndef CYBERDOG_TOUCH__CYBERDOG_TOUCH_HPP_
#define CYBERDOG_TOUCH__CYBERDOG_TOUCH_HPP_
#include <string>
#include <functional>
#include "rclcpp/rclcpp.hpp"
#include "protocol/msg/touch_status.hpp"

namespace cyberdog
{
namespace device
{
class TouchBase
{
  using TouchStatusMsg = protocol::msg::TouchStatus;
protected:
  TouchBase();
public:
  virtual bool Config() = 0;
  virtual bool Init(std::function<void(TouchStatusMsg)> f) = 0;
  virtual bool SelfCheck() = 0;
  virtual bool RegisterTopic(std::function<void(TouchStatusMsg)> f) = 0;  
};  // class TouchBasecd
}  // namespace device
}  // namespace cyberdog
#endif  // CYBERDOG_TOUCH__CYBERDOG_TOUCH_HPP_
