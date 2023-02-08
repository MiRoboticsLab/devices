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
#include <utility>
#include "rclcpp/rclcpp.hpp"
#include "ament_index_cpp/get_package_share_directory.hpp"
#include "protocol/srv/led_execute.hpp"
#include "embed_protocol/embed_protocol.hpp"
#include "cyberdog_system/robot_code.hpp"
#include "cyberdog_led/led_base.hpp"
#include "cyberdog_led/led_plugin.hpp"
#include "cyberdog_common/cyberdog_log.hpp"
#include "cyberdog_common/cyberdog_toml.hpp"


bool cyberdog::device::LedCarpo::Init()
{
  INFO("begin led init.");
  const SYS::ModuleCode kModuleCode = SYS::ModuleCode::kLED;
  code_ = std::make_shared<SYS::CyberdogCode<LedCode>>(kModuleCode);
  bool config_res = Config();
  if (config_res == false) {
    return false;
  }
  toml::value value_table;
  auto local_share_dir = ament_index_cpp::get_package_share_directory("params");
  auto local_config_dir = local_share_dir + std::string("/toml_config/device/led_priority.toml");
  if (access(local_config_dir.c_str(), F_OK) != 0) {
    ERROR(" %s do not exist!", local_config_dir.c_str());
    return false;
  } else {
    INFO("load led_priority toml file successfully");
  }
  if (!cyberdog::common::CyberdogToml::ParseFile(local_config_dir, value_table)) {
    ERROR("fail to read data from led_priority toml");
    return false;
  }
  if (!value_table.is_table()) {
    ERROR("Toml format error");
    return false;
  }
  toml::value values;
  cyberdog::common::CyberdogToml::Get(value_table, "priority", values);
  // 初始化优先级序列
  for (uint64_t i = 0; i < values.size() - 1; i++) {
    Request_Attribute temp;
    this->headled_attrs.emplace_back(temp);
    this->tailled_attrs.emplace_back(temp);
    this->miniled_attrs.emplace_back(temp);
  }
  this->headled_attrs.emplace_back(this->system_headled);
  this->tailled_attrs.emplace_back(this->system_tailled);
  this->miniled_attrs.emplace_back(this->system_miniled);
  for (size_t i = 0; i < values.size(); i++) {
    auto value = values.at(i);
    std::string client_name;
    if (!cyberdog::common::CyberdogToml::Get(value, "client", client_name)) {
      ERROR(" fail to read key headled from toml");
    }
    uint64_t id;
    if (!cyberdog::common::CyberdogToml::Get(value, "id", id)) {
      ERROR(" fail to read key headled from toml");
    }
    this->headled_attrs[id].priority = client_name;
    this->tailled_attrs[id].priority = client_name;
    this->miniled_attrs[id].priority = client_name;
  }
  this->led_map.insert(std::make_pair(LedExecuteRequest::HEAD_LED, this->headled_attrs));
  this->led_map.insert(std::make_pair(LedExecuteRequest::TAIL_LED, this->tailled_attrs));
  this->led_map.insert(std::make_pair(LedExecuteRequest::MINI_LED, this->miniled_attrs));
  // 执行默认的头部和尾部rgb灯带系统默认灯效
  std::vector<uint8_t> temp_vector(4);
  temp_vector[0] = this->system_headled.effect;
  temp_vector[1] = this->system_headled.r_value;
  temp_vector[2] = this->system_headled.g_value;
  temp_vector[3] = this->system_headled.b_value;
  INFO("send headled system default cmd");
  this->head_can_->Operate("enable_on", temp_vector);
  INFO("send tail system default cmd");
  temp_vector[0] = this->system_tailled.effect;
  temp_vector[1] = this->system_tailled.r_value;
  temp_vector[2] = this->system_tailled.g_value;
  temp_vector[3] = this->system_tailled.b_value;
  this->tail_can_->Operate("enable_on", temp_vector);
  return true;
}
void cyberdog::device::LedCarpo::shutdown()
{
  INFO("LedCarpo shutdown");
}
// 加载一些为预定的配置文件
bool cyberdog::device::LedCarpo::Config()
{
  // head led
  auto head_toml_dir = ament_index_cpp::get_package_share_directory("params");
  auto head_config_dir = head_toml_dir + std::string("/toml_config/device/head_led.toml");
  if (access(head_config_dir.c_str(), F_OK) != 0) {
    INFO(" %s do not exist!", head_config_dir.c_str());
    return false;
  } else {
    INFO(
      "local_config_dir= %s",
      head_config_dir.c_str());
    this->head_can_ = std::make_shared<cyberdog::embed::Protocol<LedToml>>(
      head_config_dir, false);
    this->head_can_->LINK_VAR(this->head_can_->GetData()->enable_on_ack);
    this->head_can_->SetDataCallback(
      std::bind(
        &cyberdog::device::LedCarpo::head_led_callback,
        this, std::placeholders::_1, std::placeholders::_2));
  }
  // tail led
  auto tail_toml_dir = ament_index_cpp::get_package_share_directory("params");
  auto tail_config_dir = tail_toml_dir + std::string("/toml_config/device/tail_led.toml");
  if (access(tail_config_dir.c_str(), F_OK) != 0) {
    INFO(" %s do not exist!", tail_config_dir.c_str());
    return false;
  } else {
    INFO(
      "tail_config_dir= %s",
      tail_config_dir.c_str());
    this->tail_can_ = std::make_shared<cyberdog::embed::Protocol<LedToml>>(
      tail_config_dir, false);
    this->tail_can_->LINK_VAR(this->tail_can_->GetData()->enable_on_ack);
    this->tail_can_->SetDataCallback(
      std::bind(
        &cyberdog::device::LedCarpo::tail_led_callback,
        this, std::placeholders::_1, std::placeholders::_2));
  }
  // mini led
  auto mini_toml_dir = ament_index_cpp::get_package_share_directory("params");
  auto mini_config_dir = mini_toml_dir + std::string("/toml_config/device/mini_led.toml");
  if (access(mini_config_dir.c_str(), F_OK) != 0) {
    INFO(" %s do not exist!", mini_config_dir.c_str());
    return false;
  } else {
    INFO(
      "mini_config_dir= %s",
      mini_config_dir.c_str());
    this->mini_can_ = std::make_shared<cyberdog::embed::Protocol<LedToml>>(
      mini_config_dir, false);
    this->mini_can_->LINK_VAR(this->mini_can_->GetData()->enable_on_ack);
    this->mini_can_->SetDataCallback(
      std::bind(
        &cyberdog::device::LedCarpo::mini_led_callback,
        this, std::placeholders::_1, std::placeholders::_2));
  }

  // read color  and system led effect config in toml
  toml::value value;
  auto share_dir = ament_index_cpp::get_package_share_directory("params");
  auto config_dir = share_dir + std::string("/toml_config/device/led_color_config.toml");
  if (access(config_dir.c_str(), F_OK) != 0) {
    ERROR(" %s do not exist!", config_dir.c_str());
  } else {
    INFO("load led_colcor_config toml file successfully");
  }
  if (!cyberdog::common::CyberdogToml::ParseFile(config_dir, value)) {
    ERROR("fail to read data from led_colcor_config toml");
  }
  // read color
  if (!cyberdog::common::CyberdogToml::Get(value, "red", this->red)) {
    ERROR(" fail to read key red from toml");
  }
  if (!cyberdog::common::CyberdogToml::Get(value, "yellow", this->yellow)) {
    ERROR(" fail to read key yellow from toml");
  }
  if (!cyberdog::common::CyberdogToml::Get(value, "blue", this->blue)) {
    ERROR(" fail to read key blue from toml");
  }
  // read system led effect
  toml::value headled;
  if (!cyberdog::common::CyberdogToml::Get(value, "system_headled", headled)) {
    ERROR(" fail to read table system_headled from toml");
  }
  if (!cyberdog::common::CyberdogToml::Get(
      headled, "occupation",
      this->system_headled.occupation))
  {
    ERROR(" fail to read key occupation from table");
  }
  if (!cyberdog::common::CyberdogToml::Get(
      headled, "client",
      this->system_headled.client))
  {
    ERROR(" fail to read key client from table");
  }
  if (!cyberdog::common::CyberdogToml::Get(
      headled, "target",
      this->system_headled.target))
  {
    ERROR(" fail to read key target from table");
  }
  if (!cyberdog::common::CyberdogToml::Get(
      headled, "mode",
      this->system_headled.mode))
  {
    ERROR(" fail to read key mode from table");
  }
  if (!cyberdog::common::CyberdogToml::Get(
      headled, "effect",
      this->system_headled.effect))
  {
    ERROR(" fail to read key effect from table");
  }
  if (!cyberdog::common::CyberdogToml::Get(
      headled, "r_value",
      this->system_headled.r_value))
  {
    ERROR(" fail to read key r_value from table");
  }
  if (!cyberdog::common::CyberdogToml::Get(
      headled, "g_value",
      this->system_headled.g_value))
  {
    ERROR(" fail to read key g_value from table");
  }
  if (!cyberdog::common::CyberdogToml::Get(
      headled, "b_value",
      this->system_headled.b_value))
  {
    ERROR(" fail to read key b_value from table");
  }
  this->system_headled.priority = this->system_headled.client;
  toml::value tailled;
  if (!cyberdog::common::CyberdogToml::Get(value, "system_tailled", tailled)) {
    ERROR(" fail to read table system_tailled from toml");
  }
  if (!cyberdog::common::CyberdogToml::Get(
      tailled, "occupation",
      this->system_tailled.occupation))
  {
    ERROR(" fail to read key occupation from table");
  }
  if (!cyberdog::common::CyberdogToml::Get(
      tailled, "client",
      this->system_tailled.client))
  {
    ERROR(" fail to read key client from table");
  }
  if (!cyberdog::common::CyberdogToml::Get(
      tailled, "target",
      this->system_tailled.target))
  {
    ERROR(" fail to read key target from table");
  }
  if (!cyberdog::common::CyberdogToml::Get(
      tailled, "mode",
      this->system_tailled.mode))
  {
    ERROR(" fail to read key mode from table");
  }
  if (!cyberdog::common::CyberdogToml::Get(
      tailled, "effect",
      this->system_tailled.effect))
  {
    ERROR(" fail to read key effect from table");
  }
  if (!cyberdog::common::CyberdogToml::Get(
      tailled, "r_value",
      this->system_tailled.r_value))
  {
    ERROR(" fail to read key r_value from table");
  }
  if (!cyberdog::common::CyberdogToml::Get(
      tailled, "g_value",
      this->system_tailled.g_value))
  {
    ERROR(" fail to read key g_value from table");
  }
  if (!cyberdog::common::CyberdogToml::Get(
      tailled, "b_value",
      this->system_tailled.b_value))
  {
    ERROR(" fail to read key b_value from table");
  }
  this->system_tailled.priority = this->system_tailled.client;
  toml::value miniled;
  if (!cyberdog::common::CyberdogToml::Get(value, "system_miniled", miniled)) {
    ERROR(" fail to read table system_miniled from toml");
  }
  if (!cyberdog::common::CyberdogToml::Get(
      miniled, "occupation",
      this->system_miniled.occupation))
  {
    ERROR(" fail to read key occupation from table");
  }
  if (!cyberdog::common::CyberdogToml::Get(
      miniled, "client",
      this->system_miniled.client))
  {
    ERROR(" fail to read key client from table");
  }
  if (!cyberdog::common::CyberdogToml::Get(
      miniled, "target",
      this->system_miniled.target))
  {
    ERROR(" fail to read key target from table");
  }
  if (!cyberdog::common::CyberdogToml::Get(
      miniled, "mode",
      this->system_miniled.mode))
  {
    ERROR(" fail to read key mode from table");
  }
  if (!cyberdog::common::CyberdogToml::Get(
      miniled, "effect",
      this->system_miniled.effect))
  {
    ERROR(" fail to read key effect from table");
  }
  if (!cyberdog::common::CyberdogToml::Get(
      miniled, "r_value",
      this->system_miniled.r_value))
  {
    ERROR(" fail to read key r_value from table");
  }
  if (!cyberdog::common::CyberdogToml::Get(
      miniled, "g_value",
      this->system_miniled.g_value))
  {
    ERROR(" fail to read key g_value from table");
  }
  if (!cyberdog::common::CyberdogToml::Get(
      miniled, "b_value",
      this->system_miniled.b_value))
  {
    ERROR(" fail to read key b_value from table");
  }
  this->system_miniled.priority = this->system_miniled.client;
  return true;
}
// 自查下状态
int32_t cyberdog::device::LedCarpo::SelfCheck()
{
  return code_->GetKeyCode(SYS::KeyCode::kOK);
}
bool cyberdog::device::LedCarpo::request_load_priority(
  const std::shared_ptr<protocol::srv::LedExecute::Request> info_request)
{
  uint64_t index = 0;
  bool priority_ok = true;
  std::map<uint8_t, std::vector<Request_Attribute>>::iterator it;
  it = this->led_map.find(info_request->target);
  if (it == this->led_map.end()) {
    ERROR("empty target parameter or is not in the optional list");
  }
  for (uint64_t i = 0; i < it->second.size(); i++) {
    if (info_request->client == it->second[i].priority) {
      index = i;
      it->second[i].occupation = info_request->occupation;
      it->second[i].client = info_request->client;
      it->second[i].target = info_request->target;
      it->second[i].mode = info_request->mode;
      it->second[i].effect = info_request->effect;
      it->second[i].r_value = info_request->r_value;
      it->second[i].g_value = info_request->g_value;
      it->second[i].b_value = info_request->b_value;
      break;
    }
  }
  // judge priority is ok or not
  for (uint64_t j = 0; j < index; j++) {
    if (it->second[j].occupation == true) {
      priority_ok = false;
      break;
    }
  }
  return priority_ok;
}

void cyberdog::device::LedCarpo::Play(
  const std::shared_ptr<protocol::srv::LedExecute::Request> info_request,
  std::shared_ptr<protocol::srv::LedExecute::Response> info_response)
{
  // Determine whether the request parameters are legal or not
  int32_t params_islegal = request_legal(info_request);
  if (params_islegal >= LedExecuteResponse::TARGET_ERROR &&
    params_islegal <= LedExecuteResponse::EFFECT_ERROR)
  {
    WARN("request params is illegal,please reset params");
    info_response->code = params_islegal;
    return;
  } else {
    INFO("request params is legal");
  }
  // load request and judge priority
  bool priority_ok = request_load_priority(info_request);
  if (priority_ok == false) {
    WARN("request priority is not enough to play led");
    info_response->code = LedExecuteResponse::LOW_PRIORITY;
    return;
  } else {
    INFO("request priority is enough to play led");
  }
  // Play by priority
  INFO("Play by priority");
  find_cmd(info_request);
  info_response->code = play_by_priority(info_request);
}

int32_t cyberdog::device::LedCarpo::request_legal(
  const std::shared_ptr<protocol::srv::LedExecute::Request> info_request)
{
  INFO(
    "request:  occupation: %d" ", client: %s" ", target: %d"
    ", mode : %d" ", effect: %d" ", r_value: %d" ", g_value: %d"
    ", b_value: %d ",
    static_cast<int>(info_request->occupation),
    info_request->client.c_str(), static_cast<int>(info_request->target),
    static_cast<int>(info_request->mode),
    static_cast<int>(info_request->effect), static_cast<int>(info_request->r_value),
    static_cast<int>(info_request->g_value),
    static_cast<int>(info_request->b_value));
  // target
  std::map<uint8_t, std::vector<Request_Attribute>>::iterator it;
  it = this->led_map.find(info_request->target);
  if (it == this->led_map.end()) {
    ERROR("target parameter is empty or not in the optional list");
    return LedExecuteResponse::TARGET_ERROR;
  }
  // priority
  bool priority_is_legal = false;
  for (uint64_t i = 0; i < it->second.size(); i++) {
    if (info_request->client == it->second[i].priority) {
      priority_is_legal = true;
      break;
    }
  }
  if (priority_is_legal == false) {
    ERROR("client parameter is empty or not in the preset priority list");
    return LedExecuteResponse::PRIORITY_ERROR;
  }
  // mode
  if (info_request->occupation == false) {
    return 0;
  }
  if (info_request->mode != LedExecuteRequest::SYSTEM_PREDEFINED &&
    info_request->mode != LedExecuteRequest::USER_DEFINED)
  {
    ERROR("rgb/mini led only support USER_DEFINED or SYSTEM_PREDEFINED mode");
    return LedExecuteResponse::MODE_ERROR;
  }
  // effect
  bool effect_is_legal;
  if (info_request->target == LedExecuteRequest::MINI_LED) {
    if (info_request->mode == LedExecuteRequest::USER_DEFINED) {
      effect_is_legal = (info_request->effect >= LedExecuteRequest::CIRCULAR_BREATH &&
        info_request->effect <= LedExecuteRequest::CIRCULAR_RING);
      if (effect_is_legal == false) {
        ERROR(
          "mini led only support effect from CIRCULAR_BREATH"
          "to CIRCULAR_RING when mode is USER_DEFINED");
        return LedExecuteResponse::EFFECT_ERROR;
      }
    } else {
      effect_is_legal = (info_request->effect >= LedExecuteRequest::MINI_OFF &&
        info_request->effect <= LedExecuteRequest::COLOR_ONE_BY_ONE);
      if (effect_is_legal == false) {
        ERROR(
          "mini led only support effect from MINI_OFF"
          "to COLOR_ONE_BY_ONE when mode is SYSTEM_PREDEFINED");
        return LedExecuteResponse::EFFECT_ERROR;
      }
    }
  } else {
    if (info_request->mode == LedExecuteRequest::USER_DEFINED) {
      effect_is_legal = (info_request->effect >= LedExecuteRequest::RGB_ON &&
        info_request->effect <= LedExecuteRequest::TRAILING_RACE);
      if (effect_is_legal == false) {
        ERROR(
          "rgb led effect only support effect from ON to ONE_BY_ONE_FAST"
          " effect when mode is USER_DEFINED");
        return LedExecuteResponse::EFFECT_ERROR;
      }
    } else {
      effect_is_legal = (info_request->effect >= LedExecuteRequest::RGB_OFF &&
        info_request->effect <= LedExecuteRequest::YELLOW_ONE_BY_ONE_FAST);
      if (effect_is_legal == false) {
        ERROR(
          "rgb led effect only support effecy from RGB_OFF to YELLOW_ONE_BY_ONE_FAST"
          " effect when mode is SYSTEM_PREDEFINED ");
        return LedExecuteResponse::EFFECT_ERROR;
      }
    }
  }
  return 0;
}

void cyberdog::device::LedCarpo::find_cmd(
  const
  std::shared_ptr<protocol::srv::LedExecute::Request> info_request)
{
  // priority
  uint64_t index = -1;
  std::map<uint8_t, std::vector<Request_Attribute>>::iterator it;
  it = this->led_map.find(info_request->target);
  if (it == this->led_map.end()) {
    ERROR("target parameter is empty or not in the optional list");
  }

  // priority
  for (uint64_t i = 0; i < it->second.size(); i++) {
    if (info_request->client == it->second[i].priority) {
      index = i;
      if (it->second[i].occupation == true) {
        index = i;
        break;
      } else {
        for (uint64_t j = i; j < it->second.size(); j++) {
          if (it->second[j].occupation == true) {
            index = j;
            break;
          }
        }
      }
      break;
    }
  }
  // operate
  this->operatecmd = it->second[index];
  INFO("the client %s gets permission to execute led", this->operatecmd.client.c_str());
  INFO(
    "execute cmd: priority: %s" ", occupation: %d" ",client: %s" ", target: %d"
    ", mode : %d" ", effect: %d" ", r_value: %d" ", g_value: %d"
    ", b_value: %d ", this->operatecmd.priority.c_str(),
    static_cast<int>(this->operatecmd.occupation),
    this->operatecmd.client.c_str(), static_cast<int>(this->operatecmd.target),
    static_cast<int>(this->operatecmd.mode),
    static_cast<int>(this->operatecmd.effect), static_cast<int>(this->operatecmd.r_value),
    static_cast<int>(this->operatecmd.g_value),
    static_cast<int>(this->operatecmd.b_value));
}

void cyberdog::device::LedCarpo::mini_led_cmd(std::vector<uint8_t> & temp_vector)
{
  switch (this->operatecmd.effect) {
    case LedExecuteRequest::MINI_OFF: {
        temp_vector[0] = 0x00;  // off canid
        temp_vector[1] = 0;
        temp_vector[2] = 0;
        temp_vector[3] = 0;
        break;
      }
    case LedExecuteRequest::CIRCULAR_BREATH: {
        temp_vector[0] = 0x01;
        temp_vector[1] = this->operatecmd.r_value;
        temp_vector[2] = this->operatecmd.g_value;
        temp_vector[3] = this->operatecmd.b_value;
        break;
      }
    case LedExecuteRequest::CIRCULAR_RING: {
        temp_vector[0] = 0x02;
        temp_vector[1] = this->operatecmd.r_value;
        temp_vector[2] = this->operatecmd.g_value;
        temp_vector[3] = this->operatecmd.b_value;
        break;
      }
    case LedExecuteRequest::RECTANGLE_COLOR: {
        temp_vector[0] = 0x21;
        temp_vector[1] = this->operatecmd.r_value;
        temp_vector[2] = this->operatecmd.g_value;
        temp_vector[3] = this->operatecmd.b_value;
        break;
      }
    case LedExecuteRequest::CENTRE_COLOR: {
        temp_vector[0] = 0x22;
        temp_vector[1] = this->operatecmd.r_value;
        temp_vector[2] = this->operatecmd.g_value;
        temp_vector[3] = this->operatecmd.b_value;
        break;
      }
    case LedExecuteRequest::THREE_CIRCULAR: {
        temp_vector[0] = 0x23;
        temp_vector[1] = this->operatecmd.r_value;
        temp_vector[2] = this->operatecmd.g_value;
        temp_vector[3] = this->operatecmd.b_value;
        break;
      }
    case LedExecuteRequest::COLOR_ONE_BY_ONE: {
        temp_vector[0] = 0x24;
        temp_vector[1] = this->operatecmd.r_value;
        temp_vector[2] = this->operatecmd.g_value;
        temp_vector[3] = this->operatecmd.b_value;
        break;
      }
    default:
      break;
  }
}


void cyberdog::device::LedCarpo::rgb_led_cmd(std::vector<uint8_t> & temp_vector)
{
  if (this->operatecmd.mode == LedExecuteRequest::USER_DEFINED) {
    INFO("rgb led USER_DEFINED mode");
    temp_vector[0] = this->operatecmd.effect;
    temp_vector[1] = this->operatecmd.r_value;
    temp_vector[2] = this->operatecmd.g_value;
    temp_vector[3] = this->operatecmd.b_value;
  } else {
    INFO("rgb led SYSTEM_PREDEFINED mode");
    switch (this->operatecmd.effect) {
      // OFF
      case LedExecuteRequest::RGB_OFF: {
          temp_vector[0] = 0x00;
          temp_vector[1] = 0;
          temp_vector[2] = 0;
          temp_vector[3] = 0;
          break;
        }
      // RED
      case LedExecuteRequest::RED_ON: {
          temp_vector[0] = LedExecuteRequest::RGB_ON;
          temp_vector[1] = this->red[0];
          temp_vector[2] = this->red[1];
          temp_vector[3] = this->red[2];
          break;
        }
      case LedExecuteRequest::RED_BLINK: {
          temp_vector[0] = LedExecuteRequest::BLINK;
          temp_vector[1] = this->red[0];
          temp_vector[2] = this->red[1];
          temp_vector[3] = this->red[2];
          break;
        }
      case LedExecuteRequest::RED_BLINK_FAST: {
          temp_vector[0] = LedExecuteRequest::BLINK_FAST;
          temp_vector[1] = this->red[0];
          temp_vector[2] = this->red[1];
          temp_vector[3] = this->red[2];
          break;
        }
      case LedExecuteRequest::RED_BREATH: {
          temp_vector[0] = LedExecuteRequest::BREATH;
          temp_vector[1] = this->red[0];
          temp_vector[2] = this->red[1];
          temp_vector[3] = this->red[2];
          break;
        }
      case LedExecuteRequest::RED_BREATH_FAST: {
          temp_vector[0] = LedExecuteRequest::BREATH_FAST;
          temp_vector[1] = this->red[0];
          temp_vector[2] = this->red[1];
          temp_vector[3] = this->red[2];
          break;
        }
      case LedExecuteRequest::RED_ONE_BY_ONE: {
          temp_vector[0] = LedExecuteRequest::ONE_BY_ONE;
          temp_vector[1] = this->red[0];
          temp_vector[2] = this->red[1];
          temp_vector[3] = this->red[2];
          break;
        }
      case LedExecuteRequest::RED_ONE_BY_ONE_FAST: {
          temp_vector[0] = LedExecuteRequest::ONE_BY_ONE_FAST;
          temp_vector[1] = this->red[0];
          temp_vector[2] = this->red[1];
          temp_vector[3] = this->red[2];
          break;
        }
      // BLUE
      case LedExecuteRequest::BLUE_ON: {
          temp_vector[0] = LedExecuteRequest::RGB_ON;
          temp_vector[1] = this->blue[0];
          temp_vector[2] = this->blue[1];
          temp_vector[3] = this->blue[2];
          break;
        }
      case LedExecuteRequest::BLUE_BLINK: {
          temp_vector[0] = LedExecuteRequest::BLINK;
          temp_vector[1] = this->blue[0];
          temp_vector[2] = this->blue[1];
          temp_vector[3] = this->blue[2];
          break;
        }
      case LedExecuteRequest::BLUE_BLINK_FAST: {
          temp_vector[0] = LedExecuteRequest::BLINK_FAST;
          temp_vector[1] = this->blue[0];
          temp_vector[2] = this->blue[1];
          temp_vector[3] = this->blue[2];
          break;
        }
      case LedExecuteRequest::BLUE_BREATH: {
          temp_vector[0] = LedExecuteRequest::BREATH;
          temp_vector[1] = this->blue[0];
          temp_vector[2] = this->blue[1];
          temp_vector[3] = this->blue[2];
          break;
        }
      case LedExecuteRequest::BLUE_BREATH_FAST: {
          temp_vector[0] = LedExecuteRequest::BREATH_FAST;
          temp_vector[1] = this->blue[0];
          temp_vector[2] = this->blue[1];
          temp_vector[3] = this->blue[2];
          break;
        }
      case LedExecuteRequest::BLUE_ONE_BY_ONE: {
          temp_vector[0] = LedExecuteRequest::ONE_BY_ONE;
          temp_vector[1] = this->blue[0];
          temp_vector[2] = this->blue[1];
          temp_vector[3] = this->blue[2];
          break;
        }
      case LedExecuteRequest::BLUE_ONE_BY_ONE_FAST: {
          temp_vector[0] = LedExecuteRequest::ONE_BY_ONE_FAST;
          temp_vector[1] = this->blue[0];
          temp_vector[2] = this->blue[1];
          temp_vector[3] = this->blue[2];
          break;
        }
      // YELLOW
      case LedExecuteRequest::YELLOW_ON: {
          temp_vector[0] = LedExecuteRequest::RGB_ON;
          temp_vector[1] = this->yellow[0];
          temp_vector[2] = this->yellow[1];
          temp_vector[3] = this->yellow[2];
          break;
        }
      case LedExecuteRequest::YELLOW_BLINK: {
          temp_vector[0] = LedExecuteRequest::BLINK;
          temp_vector[1] = this->yellow[0];
          temp_vector[2] = this->yellow[1];
          temp_vector[3] = this->yellow[2];
          break;
        }
      case LedExecuteRequest::YELLOW_BLINK_FAST: {
          temp_vector[0] = LedExecuteRequest::BLINK_FAST;
          temp_vector[1] = this->yellow[0];
          temp_vector[2] = this->yellow[1];
          temp_vector[3] = this->yellow[2];
          break;
        }
      case LedExecuteRequest::YELLOW_BREATH: {
          temp_vector[0] = LedExecuteRequest::BREATH;
          temp_vector[1] = this->yellow[0];
          temp_vector[2] = this->yellow[1];
          temp_vector[3] = this->yellow[2];
          break;
        }
      case LedExecuteRequest::YELLOW_BREATH_FAST: {
          temp_vector[0] = LedExecuteRequest::BREATH_FAST;
          temp_vector[1] = this->yellow[0];
          temp_vector[2] = this->yellow[1];
          temp_vector[3] = this->yellow[2];
          break;
        }
      case LedExecuteRequest::YELLOW_ONE_BY_ONE: {
          temp_vector[0] = LedExecuteRequest::ONE_BY_ONE;
          temp_vector[1] = this->yellow[0];
          temp_vector[2] = this->yellow[1];
          temp_vector[3] = this->yellow[2];
          break;
        }
      case LedExecuteRequest::YELLOW_ONE_BY_ONE_FAST: {
          temp_vector[0] = LedExecuteRequest::ONE_BY_ONE_FAST;
          temp_vector[1] = this->yellow[0];
          temp_vector[2] = this->yellow[1];
          temp_vector[3] = this->yellow[2];
          break;
        }
    }
  }
}

int32_t cyberdog::device::LedCarpo::play_by_priority(
  const
  std::shared_ptr<protocol::srv::LedExecute::Request> info_request)
{
  std::vector<uint8_t> temp_vector(4);
  switch (info_request->target) {
    case LedExecuteRequest::HEAD_LED: {
        rgb_led_cmd(temp_vector);
        INFO("head led send can cmd");
        this->head_can_->Operate("enable_on", temp_vector);
        break;
      }
    case LedExecuteRequest::TAIL_LED: {
        rgb_led_cmd(temp_vector);
        INFO("tail led send can cmd");
        this->tail_can_->Operate("enable_on", temp_vector);
        break;
      }
    case LedExecuteRequest::MINI_LED: {
        INFO("mini led send can cmd");
        mini_led_cmd(temp_vector);
        this->mini_can_->Operate("enable_on", temp_vector);
        break;
      }
    default:
      break;
  }
  time_t time_delay = time(nullptr);
  while (this->operate_result == false && difftime(time(nullptr), time_delay) < 2.0f) {
    std::this_thread::sleep_for(std::chrono::microseconds(30000));
    INFO(
      " difftime = %2f ",
      difftime(time(nullptr), time_delay));
  }
  if (this->operate_result == false) {
    ERROR("led response timeout");
    return LedExecuteResponse::TIMEOUT;
  } else {
    INFO("led operate successfully ");
    this->operate_result = false;
    return LedExecuteResponse::SUCCEED;
  }
}

void cyberdog::device::LedCarpo::head_led_callback(
  std::string & name, std::shared_ptr<cyberdog::device::LedToml> data)
{
  (void)data;
  if (name == "enable_on_ack") {
    this->operate_result = true;
    INFO("head led get enable_on_ack callback.");
  }
}

void cyberdog::device::LedCarpo::tail_led_callback(
  std::string & name, std::shared_ptr<cyberdog::device::LedToml> data)
{
  (void)data;
  if (name == "enable_on_ack") {
    this->operate_result = true;
    INFO("tail led get enable_on_ack callback.");
  }
}

void cyberdog::device::LedCarpo::mini_led_callback(
  std::string & name, std::shared_ptr<cyberdog::device::LedToml> data)
{
  (void)data;
  if (name == "enable_on_ack") {
    this->operate_result = true;
    INFO("mini led get enable_on_ack callback.");
  }
}
bool cyberdog::device::LedCarpo::LedCarpo::request_priority()
{
  return true;
}

PLUGINLIB_EXPORT_CLASS(cyberdog::device::LedCarpo, cyberdog::device::LedBase)
