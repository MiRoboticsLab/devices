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
#ifndef DEVICE_MANAGER__DEVICE_CONFIG_HPP_
#define DEVICE_MANAGER__DEVICE_CONFIG_HPP_

#include <map>
#include <string>
#include <utility>

#include "pluginlib/class_loader.hpp"
#include "cyberdog_led/led_base.hpp"
#include "cyberdog_bms/bms_base.hpp"
#include "cyberdog_uwb/uwb_base.hpp"
#include "cyberdog_touch/touch_base.hpp"
#include "protocol/srv/led_execute.hpp"
#include "protocol/srv/bms_cmd.hpp"
#include "protocol/srv/get_uwb_mac_session_id.hpp"
#include "protocol/msg/touch_status.hpp"
#include "protocol/msg/bms_status.hpp"
#include "protocol/msg/uwb_raw.hpp"
#include "protocol/msg/uwb_array.hpp"


namespace cyberdog
{
namespace device
{
inline void GetDeviceNames(std::map<std::string, std::string> & name_map)
{
  name_map.insert(std::make_pair("led_base", "LedCarpo"));
  name_map.insert(std::make_pair("touch_base", "TouchCarpo"));
  name_map.insert(std::make_pair("bms_base", "BMSCarpo"));
  name_map.insert(std::make_pair("uwb_base", "UWBCarpo"));
}
}  // namespace device
}  // namespace cyberdog

#endif  // DEVICE_MANAGER__DEVICE_CONFIG_HPP_
