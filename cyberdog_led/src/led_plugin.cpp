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
#include <unistd.h>
#include <time.h>
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
#include "cyberdog_led/led_plugin.hpp"
#include "cyberdog_common/cyberdog_log.hpp"


bool cyberdog::device::LedCarpo::Init() 
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
  head_led_controller.led_thread = std::thread(std::bind(&LedCarpo::head_led_thread, this));
  // mini_led_controller.led_thread = std::thread(std::bind(&LedCarpo::mini_led_thread, this));
  tail_led_controller.led_thread  = std::thread(std::bind(&LedCarpo::tail_led_thread, this));
  return true;
}
  // 加载一些为预定的配置文件
bool cyberdog::device::LedCarpo::Config()  
  {
    return true;
  }
  // 自查下状态
bool cyberdog::device::LedCarpo::SelfCheck() 
  {
    return true;
  }
  // 接受client发来的指令；设置线程条件变量
void cyberdog::device::LedCarpo::Play(
    const std::shared_ptr<protocol::srv::LedExecute::Request> info_request,
    std::shared_ptr<protocol::srv::LedExecute::Response> info_response) 
  {
    new_request.mode = info_request->mode;
    new_request.client = info_request->client;
    new_request.target = info_request->target;
    new_request.priority = info_request->priority;
    new_request.runtime = info_request->runtime;
    new_request.effect = info_request->effect;
    new_request.brightness = info_request->brightness;
    new_request.code = 0;
    // std::unique_lock<std::mutex> run_mini_led_lock(mini_led_controller.led_mutex);
    {
      std::unique_lock<std::mutex> run_tail_led_lock(tail_led_controller.led_mutex);
      std::unique_lock<std::mutex> run_head_led_lock(head_led_controller.led_mutex);
    }
    switch (new_request.target) {
      case cyberdog::device::MINI_LED:
        // 判断cliet的led command是否合法
        // 若command合法，判定优先级是否
        if (new_request.runtime == protocol::srv::LedExecute::Request::COMMAND_ALWAYSON) {
          mini_led_controller.alwayson_id = new_request.effect;
        }
        if (request_legal() && request_priority()) {
          mini_led_controller.led_workable = true;
          mini_led_controller.led_waitcv.notify_all();
        } else {
          mini_led_controller.led_workable = false;
        }
        break;
      case cyberdog::device::TAIL_LED:
        // 判断cliet的led command是否合法
        // 若command合法，判定优先级是否
        if (new_request.runtime == protocol::srv::LedExecute::Request::COMMAND_ALWAYSON) {
          tail_led_controller.alwayson_id = new_request.effect;
        }
        if (request_legal() && request_priority()) {
          tail_led_controller.led_workable = true;
          tail_led_controller.led_waitcv.notify_all();
        } else {
          tail_led_controller.led_workable = false;
        }
        break;
      case cyberdog::device::HEAD_LED:
        // 判断cliet的led command是否合法
        // 若command合法，判定优先级是否
        if (new_request.runtime == protocol::srv::LedExecute::Request::COMMAND_ALWAYSON) {
          head_led_controller.alwayson_id = new_request.effect;
        }
        if (request_legal() && request_priority()) {
          head_led_controller.led_workable = true;
          head_led_controller.led_waitcv.notify_all();
        } else {
          head_led_controller.led_workable = false;
        }
        break;
      default:
        break;
    }

    time_t time_rear = time(nullptr);
    while (new_request.code == 0 && difftime(time(nullptr), time_rear) < 2.0f) {
      std::this_thread::sleep_for(std::chrono::microseconds(30000));
      RCLCPP_INFO(rclcpp::get_logger("cyberdog_led"), "new_request.code = %d",
        new_request.code);
    }
    RCLCPP_INFO(rclcpp::get_logger("cyberdog_led"), "info_response code = %d",
      new_request.code); 
    info_response->code = new_request.code;
    new_request.code = 0;
}
bool cyberdog::device::LedCarpo::request_legal()
{
  return true;
}
bool cyberdog::device::LedCarpo::LedCarpo::request_priority()
{
  return true;
}

void cyberdog::device::LedCarpo::tail_led_thread()
{
  RCLCPP_INFO(rclcpp::get_logger("cyberdog_led"), "begin tail_led_thread.");
  RCLCPP_INFO(rclcpp::get_logger("cyberdog_led"), "tail_led_thread dump.");
  std::unique_lock<std::mutex> run_tail_led_lock(tail_led_controller.led_mutex);
  auto local_share_dir = ament_index_cpp::get_package_share_directory("params");
  auto local_config_dir = local_share_dir + std::string("/toml_config/device/tail_led.toml");
  if (access(local_config_dir.c_str(), F_OK) != 0) {
    RCLCPP_INFO(rclcpp::get_logger("cyberdog_led"), " %s do not exist!", local_config_dir.c_str());
  } else {
    RCLCPP_INFO(
      rclcpp::get_logger("cyberdog_led"), "local_config_dir= %s",
      local_config_dir.c_str());
    auto tail_led_can_ = std::make_shared<cyberdog::embed::Protocol<TailLed>>(
      local_config_dir, false);
    while (!ready) {
      tail_led_controller.led_waitcv.wait(
        run_tail_led_lock, [&] {
          return tail_led_controller.led_workable;
        });
      if (tail_led_controller.led_workable == true) {
        // can发送指令进行灯效设计
        if (new_request.mode == protocol::srv::LedExecute::Request::SET_EXISTED_EFFECT) {
          tail_led_can_->Operate("enable_on", std::vector<uint8_t>
            {new_request.mode , new_request.effect});
          RCLCPP_INFO(
            rclcpp::get_logger(
              "cyberdog_led"), "began run tail_led_thread !! request: client: %d" ", target: %d"
            ", priority: %d" ", runtime: %lu" ", effect: %d" ".  run over!!!",
            new_request.client, new_request.target, new_request.priority,
            new_request.runtime, new_request.effect);
          tail_led_controller.led_workable = false;  
        }
      }
      tail_led_controller.led_waitcv.wait_for(run_tail_led_lock, 
        std::chrono::milliseconds(new_request.runtime));
      if (tail_led_controller.led_workable == true) {
        RCLCPP_INFO(rclcpp::get_logger("cyberdog_led"), " new tail_led_thread command come in .");
      } else {
        RCLCPP_INFO(
          rclcpp::get_logger(
            "cyberdog_led"), "tail_led_thread request:  client: %d" ", target: %d"
          ", priority: %d" ", runtime: %lu" ", effect: %d" ".  run over!!!",
          new_request.client, new_request.target, new_request.priority,
          new_request.runtime, new_request.effect);
        if (!tail_led_controller.alwayson_id !=0 && 
          new_request.effect != protocol::srv::LedExecute::Request::OFF) {
          // 继续执行上一条长亮指令
          tail_led_can_->Operate("enable_on", 
            std::vector<uint8_t>{0x00,tail_led_controller.alwayson_id});
          RCLCPP_INFO(
            rclcpp::get_logger(
              "cyberdog_led"), " tail_led_alwayson_id=" " : %d", tail_led_controller.alwayson_id);
          RCLCPP_INFO(
            rclcpp::get_logger(
              "cyberdog_led"),
            " alwayson tail_led_thread comand running ,tail_led_thread  dump again! ");
        } else {
          tail_led_can_->Operate("enable_on", 
            std::vector<uint8_t>{0x00,protocol::srv::LedExecute::Request::OFF});
        }
      }
    }
  }
}


void cyberdog::device::LedCarpo::head_led_thread()
{
  RCLCPP_INFO(rclcpp::get_logger("cyberdog_led"), "begin head_led_thread.");
  RCLCPP_INFO(rclcpp::get_logger("cyberdog_led"), "head_led_thread dump.");
  std::unique_lock<std::mutex> run_head_led_lock(head_led_controller.led_mutex);
  auto local_share_dir = ament_index_cpp::get_package_share_directory("params");
  auto local_config_dir = local_share_dir + std::string("/toml_config/device/head_led.toml");
  if (access(local_config_dir.c_str(), F_OK) != 0) {
    RCLCPP_INFO(rclcpp::get_logger("cyberdog_led"), " %s do not exist!", local_config_dir.c_str());
  } else {
    RCLCPP_INFO(
      rclcpp::get_logger("cyberdog_led"), "local_config_dir= %s",
      local_config_dir.c_str());
    auto head_led_can_ = std::make_shared<cyberdog::embed::Protocol<HeadLed>>(
      local_config_dir, false);
    head_led_can_->SetDataCallback(
      std::bind(
        &cyberdog::device::LedCarpo::head_led_callback,
        this, std::placeholders::_1, std::placeholders::_2));
    head_led_can_->LINK_VAR(head_led_can_->GetData()->enable_on_ack);
    while (!ready) {
      head_led_controller.led_waitcv.wait(
        run_head_led_lock, [&] {
          return head_led_controller.led_workable;
        });
      if (head_led_controller.led_workable == true) {
        // can发送指令进行灯效设计
        if (new_request.mode == protocol::srv::LedExecute::Request::SET_EXISTED_EFFECT) {
          std::vector<uint8_t> temp_vector(2);
          temp_vector[0] = new_request.mode;
          temp_vector[1] = new_request.effect;
          head_led_can_->Operate("enable_on",temp_vector);
          RCLCPP_INFO(
            rclcpp::get_logger(
              "cyberdog_led"), "began run head_led_thread !! request: client: %d" ", target: %d"
            ", priority: %d" ", runtime: %lu" ", effect: %d" ".",
            new_request.client, new_request.target, new_request.priority,
            new_request.runtime, new_request.effect);
          head_led_controller.led_workable = false;  
        }
      }
      head_led_controller.led_waitcv.wait_for(run_head_led_lock, 
        std::chrono::milliseconds(new_request.runtime));
      if (head_led_controller.led_workable == true) {
        RCLCPP_INFO(rclcpp::get_logger("cyberdog_led"), " new head_led_thread command come in .");
      } else {
        RCLCPP_INFO(
          rclcpp::get_logger(
            "cyberdog_led"), "head_led_thread request:  client: %d" ", target: %d"
          ", priority: %d" ", runtime: %lu" ", effect: %d" ".  run over!!!",
          new_request.client, new_request.target, new_request.priority,
          new_request.runtime, new_request.effect);
        if (!head_led_controller.alwayson_id !=0 && 
          new_request.effect != protocol::srv::LedExecute::Request::OFF) {
          // 继续执行上一条长亮指令
          head_led_can_->Operate("enable_on", 
            std::vector<uint8_t>{0x00,head_led_controller.alwayson_id});
          RCLCPP_INFO(
            rclcpp::get_logger(
              "cyberdog_led"), " tail_led_alwayson_id=" " : %d", head_led_controller.alwayson_id);
          RCLCPP_INFO(
            rclcpp::get_logger(
              "cyberdog_led"),
            " alwayson head_led_thread comand running ,head_led_thread  dump again! ");
        } else {
          head_led_can_->Operate("enable_on", 
            std::vector<uint8_t>{0x00,protocol::srv::LedExecute::Request::OFF});
        }
      }
    }
  }
}
void cyberdog::device::LedCarpo::head_led_callback(
    std::string & name, std::shared_ptr<cyberdog::device::HeadLed> data)
{
  RCLCPP_INFO(rclcpp::get_logger("cyberdog_led"), "head led callback.");
  if (name == "enable_on_ack") {
    new_request.code = 1;
    RCLCPP_INFO(rclcpp::get_logger("cyberdog_led"), "head led callback name enable_on_ack.");
  }
}

/*
void cyberdog::device::LedCarpo::mini_led_thread()
{
  RCLCPP_INFO(rclcpp::get_logger("cyberdog_led"), "begin mini_led_thread.");
  RCLCPP_INFO(rclcpp::get_logger("cyberdog_led"), "mini_led_thread dump.");
  std::unique_lock<std::mutex> run_mini_led_lock(mini_led_run_mutex);
  auto local_share_dir = ament_index_cpp::get_package_share_directory("params");
  auto local_config_dir = local_share_dir + std::string("/toml_config/device/mini_led.toml");
  const char * char_disable = "enable_off";
  std::string str_disable(char_disable);
  if (access(local_config_dir.c_str(), F_OK) != 0) {
    RCLCPP_INFO(rclcpp::get_logger("cyberdog_led"), " %s do not exist!", local_config_dir.c_str());
  } else {
    RCLCPP_INFO(
      rclcpp::get_logger("cyberdog_led"), "local_config_dir= %s",
      local_config_dir.c_str());
    auto mini_led_can_ = std::make_shared<cyberdog::embed::Protocol<MiniLed>>(
      local_config_dir, false);
    while (!ready) {
      mini_led_waitcv.wait(
        run_mini_led_lock, [&] {
          return mini_led_workable;
        });
      if (mini_led_workable == true) {
        // can发送指令进行灯效设计
        // std::cout << new_request.timeout << std::endl;
        mini_led_can_->Operate(new_request.effect, std::vector<uint8_t>{});
        RCLCPP_INFO(
          rclcpp::get_logger(
            "cyberdog_led"), "began run mini_led_thread !! request:  client: %s" ", target: %s"
          ", priority: %d" ", timeout: %lu" ", effect: %s",
          (new_request.client).c_str(), (new_request.target).c_str(), new_request.priority,
          new_request.timeout, (new_request.effect).c_str());
        mini_led_workable = false;
      }
      mini_led_waitcv.wait_for(run_mini_led_lock, std::chrono::milliseconds(new_request.timeout));
      if (mini_led_workable == true) {
        RCLCPP_INFO(rclcpp::get_logger("cyberdog_led"), " new mini_led_thread command come in .");
      } else {
        RCLCPP_INFO(
          rclcpp::get_logger(
            "cyberdog_led"), "mini_led_thread request:  client: %s" ", target: %s"
          ", priority: %d" ", timeout: %lu" ", effect: %s" ".  run over!!!",
          (new_request.client).c_str(), (new_request.target).c_str(), new_request.priority,
          new_request.timeout, (new_request.effect).c_str());
        // 继续执行上一条长亮指令
        if (!mini_led_alwayson_id.empty() && new_request.effect != "enable_off") {
          mini_led_can_->Operate(mini_led_alwayson_id, std::vector<uint8_t>{});
          RCLCPP_INFO(
            rclcpp::get_logger(
              "cyberdog_led"),
            " alwayson mini_led_thread comand running ,mini_led_thread  dump again! ");
        } else {
          mini_led_can_->Operate("enable_off", std::vector<uint8_t>{});
        }
      }
    }
  }
}
*/
PLUGINLIB_EXPORT_CLASS(cyberdog::device::LedCarpo, cyberdog::device::LedBase)
