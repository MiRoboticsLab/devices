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
#include <pluginlib/class_list_macros.hpp>
#include <mutex>
#include <thread>
#include <condition_variable>
#include <string>
#include <iostream>
#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "ament_index_cpp/get_package_share_directory.hpp"
#include "protocol/srv/led_execute.hpp"
#include "embed_protocol/embed_protocol.hpp"
#include "cyberdog_system/robot_code.hpp"
#include "cyberdog_led/led_base.hpp"


#define MINI_LED 1
#define TAIL_LED 2
typedef struct led_request
{
  std::string client;
  uint8_t target;
  uint8_t priority;
  uint64_t timeout;
  uint8_t effect;
  int32_t code;
} led_request;
// 尾灯的toml映射
class TailLed
{
public:
  uint8_t effect_id;
};

class MiniLed
{
public:
  uint8_t effect_id;
};


namespace cyberdog
{
namespace device
{

class LedCarpo : public cyberdog::device::LedBase
{
private:
  /* data */
  led_request new_request;
  bool ready = false;
  // tail_led controller
  std::condition_variable tail_led_waitcv;
  std::mutex tail_led_run_mutex;
  std::thread tail_led_run_thread;
  bool tail_led_workable = false;
  uint8_t tail_led_alwayson_id;
  // static void start_led_thread(LedCarpo* temp_ptr);
  void tail_led_thread();
  // mini_led controller
  std::condition_variable mini_led_waitcv;
  std::mutex mini_led_run_mutex;
  std::thread mini_led_run_thread;
  bool mini_led_workable = false;
  uint8_t mini_led_alwayson_id;
  // static void start_led_thread(LedCarpo* temp_ptr);
  void mini_led_thread();
  bool request_legal();
  bool request_priority();

public:
  bool Init() override
  {
    RCLCPP_INFO(rclcpp::get_logger("cyberdog_led"), "begin led and set default led id.");
    // 预留发送初始化默认灯效给mcu接口
    // 初始化toml实例
    RCLCPP_INFO(
      rclcpp::get_logger(
        "cyberdog_led"), "COMAMAND_OFF: %ld", protocol::srv::LedExecute::Request::COMMAND_OFF);
    RCLCPP_INFO(
      rclcpp::get_logger("cyberdog_led"), "COMAMAND_OFF: %lu",
      (protocol::srv::LedExecute::Request::COMMAND_ALWAYSON));
    mini_led_run_thread = std::thread(std::bind(&LedCarpo::mini_led_thread, this));
    tail_led_run_thread = std::thread(std::bind(&LedCarpo::tail_led_thread, this));
    return true;
  }
  // 加载一些为预定的配置文件
  bool Config()  override
  {
    return true;
  }
  // 自查下状态
  bool SelfCheck() override
  {
    return true;
  }
  // 接受client发来的指令；设置线程条件变量
  void Play(
    const std::shared_ptr<protocol::srv::LedExecute::Request> info_request,
    std::shared_ptr<protocol::srv::LedExecute::Response> info_response) override
  {
    new_request.client = info_request->client;
    new_request.target = info_request->target;
    new_request.priority = info_request->priority;
    new_request.timeout = info_request->timeout;
    new_request.effect = info_request->effect;
    std::unique_lock<std::mutex> run_mini_led_lock(mini_led_run_mutex);
    std::unique_lock<std::mutex> run_tail_led_lock(tail_led_run_mutex);
    switch (new_request.target) {
      case MINI_LED:
        // 判断cliet的led command是否合法
        // 若command合法，判定优先级是否
        if (request_legal() && request_priority()) {
          mini_led_workable = true;
          mini_led_waitcv.notify_all();
        } else {
          mini_led_workable = false;
        }
        if (new_request.timeout == protocol::srv::LedExecute::Request::COMMAND_ALWAYSON) {
          mini_led_alwayson_id = new_request.effect;
        }
        break;
      case TAIL_LED:
        // 判断cliet的led command是否合法
        // 若command合法，判定优先级是否
        if (request_legal() && request_priority()) {
          tail_led_workable = true;
          tail_led_waitcv.notify_all();
        } else {
          tail_led_workable = false;
        }
        if (new_request.timeout == protocol::srv::LedExecute::Request::COMMAND_ALWAYSON) {
          tail_led_alwayson_id = new_request.effect;
        }
        break;
      default:
        break;
    }
    new_request.code = (int32_t)cyberdog::system::KeyCode::kOK;
    info_response->code = new_request.code;
  }
};  // LedCarpo
bool LedCarpo::request_legal()
{
  return true;
}
bool LedCarpo::request_priority()
{
  return true;
}

void LedCarpo::tail_led_thread()
{
  RCLCPP_INFO(rclcpp::get_logger("cyberdog_led"), "begin tail_led_thread.");
  RCLCPP_INFO(rclcpp::get_logger("cyberdog_led"), "tail_led_thread dump.");
  std::unique_lock<std::mutex> run_tail_led_lock(tail_led_run_mutex);
  auto local_share_dir = ament_index_cpp::get_package_share_directory("params");
  auto local_config_dir = local_share_dir + std::string("/toml_config/device/tail_led.toml");
  RCLCPP_INFO(rclcpp::get_logger("cyberdog_led"), "local_config_dir= %s",local_config_dir.c_str());
  cyberdog::embed::Protocol<TailLed> tail_led_can(
    local_config_dir, true);
  tail_led_can.LINK_VAR(tail_led_can.GetData()->effect_id);
  auto tail_led_data = tail_led_can.GetData();

  while (!ready) {
    tail_led_waitcv.wait(
      run_tail_led_lock, [&] {
        return tail_led_workable;
      });
    if (tail_led_workable == true) {
      // can发送指令进行灯效设计
      tail_led_data->effect_id = new_request.effect;
      tail_led_can.SendSelfData();
      RCLCPP_INFO(
        rclcpp::get_logger(
          "cyberdog_led"), "began run tail_led_thread !! request:  client: %s" ", target: %d"
        ", priority: %d" ", timeout: %lu" ", effect: %d",
        (new_request.client).c_str(), new_request.target, new_request.priority, new_request.timeout,
        new_request.effect);
      tail_led_workable = false;
    }
    tail_led_waitcv.wait_for(run_tail_led_lock, std::chrono::milliseconds(new_request.timeout));
    if (tail_led_workable == true) {
      RCLCPP_INFO(rclcpp::get_logger("cyberdog_led"), " new tail_led_thread command come in .");
    } else {
      RCLCPP_INFO(
        rclcpp::get_logger(
          "cyberdog_led"), "new  tail_led_thread request:  client: %s" ", target: %d"
        ", priority: %d" ", timeout: %lu" ", effect: %u" ".  run over!!!",
        (new_request.client).c_str(), new_request.target, new_request.priority, new_request.timeout,
        new_request.effect);
      // 继续执行上一条长亮指令
      tail_led_data->effect_id = tail_led_alwayson_id;
      RCLCPP_INFO(
        rclcpp::get_logger(
          "cyberdog_led"), " tail_led_alwayson_id=" " : %d", tail_led_alwayson_id);
      tail_led_can.SendSelfData();
      RCLCPP_INFO(
        rclcpp::get_logger(
          "cyberdog_led"),
        " alwayson tail_led_thread comand running ,tail_led_thread  dump again! ");
    }
  }
}


void LedCarpo::mini_led_thread()
{
  RCLCPP_INFO(rclcpp::get_logger("cyberdog_led"), "begin mini_led_thread.");
  RCLCPP_INFO(rclcpp::get_logger("cyberdog_led"), "mini_led_thread dump.");
  std::unique_lock<std::mutex> run_mini_led_lock(mini_led_run_mutex);
  auto local_share_dir = ament_index_cpp::get_package_share_directory("params");
  auto local_config_dir = local_share_dir + std::string("/toml_config/device/mini_led.toml");
  cyberdog::embed::Protocol<MiniLed> mini_led_can(
    local_config_dir, true);
  mini_led_can.LINK_VAR(mini_led_can.GetData()->effect_id);
  auto mini_led_data = mini_led_can.GetData();
  while (!ready) {
    mini_led_waitcv.wait(
      run_mini_led_lock, [&] {
        return mini_led_workable;
      });
    if (mini_led_workable == true) {
      // can发送指令进行灯效设计
      mini_led_data->effect_id = new_request.effect;
      mini_led_can.SendSelfData();
      RCLCPP_INFO(
        rclcpp::get_logger(
          "cyberdog_led"), "began run mini_led_thread !! request:  client: %s" ", target: %d"
        ", priority: %d" ", timeout: %lu" ", effect: %u",
        (new_request.client).c_str(), new_request.target, new_request.priority, new_request.timeout,
        new_request.effect);
      mini_led_workable = false;
    }
    mini_led_waitcv.wait_for(run_mini_led_lock, std::chrono::milliseconds(new_request.timeout));
    if (mini_led_workable == true) {
      RCLCPP_INFO(rclcpp::get_logger("cyberdog_led"), " new mini_led_thread command come in .");
    } else {
      RCLCPP_INFO(
        rclcpp::get_logger(
          "cyberdog_led"), "new  mini_led_thread request:  client: %s" ", target: %d"
        ", priority: %d" ", timeout: %lu" ", effect: %u" ".  run over!!!",
        (new_request.client).c_str(), new_request.target, new_request.priority, new_request.timeout,
        new_request.effect);
      // 继续执行上一条长亮指令
      mini_led_data->effect_id = mini_led_alwayson_id;
      mini_led_can.SendSelfData();
      RCLCPP_INFO(
        rclcpp::get_logger(
          "cyberdog_led"),
        " alwayson mini_led_thread comand running ,mini_led_thread  dump again! ");
    }
  }
}

}  // namespace device

}  // namespace cyberdog
PLUGINLIB_EXPORT_CLASS(cyberdog::device::LedCarpo, cyberdog::device::LedBase)
