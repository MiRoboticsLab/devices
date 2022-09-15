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


namespace cyberdog
{
namespace device
{
  const uint8_t HEAD_LED = 0;
  const uint8_t TAIL_LED = 1;
  const uint8_t MINI_LED = 2;
  // "tail_led", "head_led","mini_led"
struct led_request
{
  uint8_t mode;         // 模式取值 见协议中常量定义
  uint8_t client;       // "bms";"bluetooth";"audio" ..;
  uint8_t target;       // "tail_led", "head_led","mini_led" ...
  uint8_t priority;     // response priority(逻辑暂缺)
  uint64_t runtime;     // 灯效执行时间，单位ms
  uint8_t effect;       // 灯效 见协议中常量定义
  uint8_t brightness;   // 亮度 0~255
  int32_t code;         // mcu灯效执行结果
  led_request()
  {
    int32_t code = 0;
  }
};
// toml映射
struct HeadLed
{
  uint8_t enable_on;
  uint8_t enable_on_ack;
};

struct TailLed
{
  uint8_t enable_on;
  uint8_t enable_on_ack;
};

struct MiniLed
{
  uint8_t enable_on_ack;
  uint8_t enable_off_ack;
  uint8_t PIC_1_ACK;
  uint8_t PIC_2_ACK;
  uint8_t PIC_ANIMATION_ACK;
};

struct LedController
{
  std::condition_variable led_waitcv;
  std::mutex led_mutex;
  std::thread led_thread;
  bool led_workable = false;
  uint8_t alwayson_id; //always on effect
};

class LedCarpo : public cyberdog::device::LedBase
{
private:
  /* data */
  led_request new_request;
  bool ready = false;
  // led controller
  LedController head_led_controller;
  LedController tail_led_controller;
  LedController mini_led_controller;
  void head_led_thread();
  void tail_led_thread();
  void mini_led_thread();
  bool request_legal();
  bool request_priority();
  void head_led_callback(std::string & name, std::shared_ptr<cyberdog::device::HeadLed> data);

  LOGGER_MINOR_INSTANCE("cyberdog_led");
public:
  bool Config() override;
  bool Init() override;
  bool SelfCheck() override;
  void Play(
    const std::shared_ptr<protocol::srv::LedExecute::Request> info_request,
    std::shared_ptr<protocol::srv::LedExecute::Response> info_response) override;
};  // class LedCarpo
}  // namespace device
}  // namespace cyberdog

#endif  // CYBERDOG_LED__LED_PLUGIN_HPP_
