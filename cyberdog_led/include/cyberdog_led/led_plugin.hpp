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
#ifndef CYBERDOG_LED__LED_PLUGIN_HPP_
#define CYBERDOG_LED__LED_PLUGIN_HPP_
#include <unistd.h>
#include <mutex>
#include <thread>
#include <vector>
#include <condition_variable>
#include <string>
#include <iostream>
#include <memory>
#include <map>
#include "rclcpp/rclcpp.hpp"
#include "ament_index_cpp/get_package_share_directory.hpp"
#include "protocol/srv/led_execute.hpp"
#include "embed_protocol/embed_protocol.hpp"
#include "cyberdog_system/robot_code.hpp"
#include "cyberdog_led/led_base.hpp"
#include "cyberdog_common/cyberdog_semaphore.hpp"


namespace cyberdog
{
namespace device
{
namespace EP = cyberdog::embed;
namespace SYS = cyberdog::system;

using LedExecuteRequest = protocol::srv::LedExecute::Request;
using LedExecuteResponse = protocol::srv::LedExecute::Response;
struct Request_Attribute
{
  std::string priority;
  bool occupation;
  bool isoff;
  std::string client;
  uint8_t target;
  uint8_t mode;
  uint8_t effect;
  uint8_t r_value;
  uint8_t g_value;
  uint8_t b_value;
  Request_Attribute()
  {
    occupation = false;
    isoff = true;
  }
};
// toml映射
struct LedToml
{
  uint8_t enable_on_ack;
};
class LedCarpo : public cyberdog::device::LedBase
{
private:
  /* data */
  bool ready = false;
  std::shared_ptr<cyberdog::embed::Protocol<LedToml>> head_can_;
  std::shared_ptr<cyberdog::embed::Protocol<LedToml>> tail_can_;
  std::shared_ptr<cyberdog::embed::Protocol<LedToml>> mini_can_;
  std::map<uint8_t, std::vector<Request_Attribute>> led_map;
  std::vector<Request_Attribute> headled_attrs;
  std::vector<Request_Attribute> tailled_attrs;
  std::vector<Request_Attribute> miniled_attrs;
  // Request_Attribute operatecmd;
  Request_Attribute system_headled;
  Request_Attribute system_tailled;
  Request_Attribute system_miniled;
  std::vector<uint8_t> red;
  std::vector<uint8_t> yellow;
  std::vector<uint8_t> blue;
  bool head_operate_result = false;
  bool tail_operate_result = false;
  bool mini_operate_result = false;
  void rgb_led_cmd(std::vector<uint8_t> & temp_vector, Request_Attribute & operatecmd);
  void mini_led_cmd(std::vector<uint8_t> & temp_vector, Request_Attribute & operatecmd);
  void find_cmd(
    const std::shared_ptr<protocol::srv::LedExecute::Request> info_request,
    Request_Attribute & operatecmd);
  int32_t request_legal(const std::shared_ptr<protocol::srv::LedExecute::Request> info_request);
  int32_t play_by_priority(Request_Attribute & operatecmd);
  bool request_priority();
  bool request_load_priority(
    const std::shared_ptr<protocol::srv::LedExecute::Request> info_request);
  void head_led_callback(std::string & name, std::shared_ptr<cyberdog::device::LedToml> data);
  void tail_led_callback(std::string & name, std::shared_ptr<cyberdog::device::LedToml> data);
  void mini_led_callback(std::string & name, std::shared_ptr<cyberdog::device::LedToml> data);
  LOGGER_MINOR_INSTANCE("cyberdog_led");

public:
  enum class LedCode : int32_t
  {
    kDemoError1 = 21
  };

public:
  std::shared_ptr<SYS::CyberdogCode<LedCode>> code_{nullptr};
  void shutdown() override;
  bool Config() override;
  bool Init() override;
  int32_t SelfCheck() override;
  void Play(
    const std::shared_ptr<protocol::srv::LedExecute::Request> info_request,
    std::shared_ptr<protocol::srv::LedExecute::Response> info_response) override;
};  // class LedCarpo
}  // namespace device
}  // namespace cyberdog

#endif  // CYBERDOG_LED__LED_PLUGIN_HPP_
