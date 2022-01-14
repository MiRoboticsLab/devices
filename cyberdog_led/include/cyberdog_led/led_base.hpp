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
#ifndef CYBERDOG_LED__LED_BASE_HPP_
#define CYBERDOG_LED__LED_BASE_HPP_
#include <string>
#include "cyberdog_system/robot_code.hpp"
#include "protocol/srv/led_execute.hpp"

namespace cyberdog
{
namespace device
{
class LedBase
{
protected:
  LedBase() {}
public:
  virtual void Config() = 0;
  virtual bool Init() = 0;
  virtual bool SelfCheck() = 0;
  virtual bool Play(const protocol::srv::LedExecute_Request::SharedPtr request,
    protocol::srv::LedExecute_Response::SharedPtr response) = 0;
};  // class LedBase
}  // namespace device
}  // namespace cyberdog

#endif  // CYBERDOG_LED__LED_BASE_HPP_