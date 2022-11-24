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

#include <memory>
#include <string>
#include <vector>
#include <limits>
#include <utility>
#include <tuple>

#include "cyberdog_uwb/uwb_plugin.hpp"
#include "cyberdog_uwb/float_comparisons.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "ament_index_cpp/get_package_share_directory.hpp"
#include "protocol/srv/get_uwb_mac_session_id.hpp"

namespace cyberdog
{
namespace device
{

UWBCarpo::UWBCarpo()
{
  queue_.resize(5);
  ros_uwb_status_.data.resize(4);
  if (!LoadUWBTomlConfig()) {
    ERROR("Load UWB parameters error.");
  }
}

UWBCarpo::~UWBCarpo()
{
  threading_ = false;
  {
    std::unique_lock<std::mutex> lock(task_checking_.mt_);
    task_checking_.cv_.notify_all();
  }
  if (uwb_thread_->joinable()) {
    uwb_thread_->join();
  }
}


bool UWBCarpo::Config()
{
  return true;
}

bool UWBCarpo::Init(
  std::function<void(UwbSignleStatusMsg)>
  function_callback, bool simulation)
{
  if (use_uwb_) {
    INFO("Current uwb device available.");
  } else {
    WARN("Current uwb device unavailable, please set enable in toml file.");
    return true;
  }

  RegisterTopic(function_callback);

  simulation_ = simulation;
  if (simulation_) {
    RunSimulation();
    initialized_finished_ = true;
    return initialized_finished_;
  } else {
    // make link for can
    initialized_finished_ = InitializeCanCommunication();
    if (!initialized_finished_) {
      INFO("Create can communication error.");
      return initialized_finished_;
    }

    // Enable
    initialized_finished_ = Initialize();
    if (!initialized_finished_) {
      INFO("Enable initialize error.");
      return initialized_finished_;
    }
  }
  threading_ = true;
  uwb_thread_ = std::make_shared<std::thread>(std::bind(&UWBCarpo::RunTask, this));
  checking_thread_ = std::make_unique<std::thread>(std::bind(&UWBCarpo::checkResponse, this));

  INFO("[UWBCarpo]: %s", "UWBCarpo initialize success.");
  return initialized_finished_;
}

bool UWBCarpo::SelfCheck()
{
  return true;
}

bool UWBCarpo::LowPower()
{
  return true;
}

bool UWBCarpo::RegisterTopic(std::function<void(UwbSignleStatusMsg)> function_callback)
{
  status_function_ = function_callback;
  return true;
}

void UWBCarpo::Play(
  const std::shared_ptr<protocol::srv::GetUWBMacSessionID::Request> info_request,
  std::shared_ptr<protocol::srv::GetUWBMacSessionID::Response> info_response)
{
  info_response->session_id = uwb_connect_info_.session_id;
  info_response->master = uwb_connect_info_.controller_mac;
  info_response->slave1 = uwb_connect_info_.head_uwb_mac;
  info_response->slave2 = uwb_connect_info_.head_tof_mac;
  info_response->slave3 = uwb_connect_info_.rear_uwb_mac;
  info_response->slave4 = uwb_connect_info_.rear_tof_mac;
  Open();
}

void UWBCarpo::SetConnectedState(bool connected)
{
  if (connected) {
    INFO("UWB is connected");
  } else {
    INFO("UWB is disconnected");
    if (activated_) {
      if (!Close()) {
        WARN("Close UWB operation failed!");
      } else {
        INFO("Close UWB operation succeeded");
      }
    }
  }
  activated_ = connected;
}

bool UWBCarpo::Open()
{
  checking_task_queue_.push(wait_open_res_.task_no_);
  if (!head_turn_on_ || !head_tof_turn_on_ || !rear_turn_on_ || !rear_tof_turn_on_) {
    head_can_ptr_->LINK_VAR(head_can_ptr_->GetData()->head_enable_on_ack);
    head_can_ptr_->LINK_VAR(head_can_ptr_->GetData()->head_tof_enable_on_ack);
    rear_can_ptr_->LINK_VAR(rear_can_ptr_->GetData()->rear_enable_on_ack);
    rear_can_ptr_->LINK_VAR(rear_can_ptr_->GetData()->rear_tof_enable_on_ack);
    head_can_ptr_->Operate("head_enable_on", std::vector<uint8_t>{});
    head_can_ptr_->Operate("head_tof_enable_on", std::vector<uint8_t>{});
    rear_can_ptr_->Operate("rear_enable_on", std::vector<uint8_t>{});
    rear_can_ptr_->Operate("rear_tof_enable_on", std::vector<uint8_t>{});
  }

  std::unique_lock<std::mutex> lock(task_checking_.mt_);
  task_checking_.cv_.notify_one();

  return true;
}

bool UWBCarpo::Close()
{
  head_can_ptr_->BREAK_VAR(head_can_ptr_->GetData()->head_data_array);
  head_can_ptr_->BREAK_VAR(head_can_ptr_->GetData()->head_tof_data_array);
  rear_can_ptr_->BREAK_VAR(rear_can_ptr_->GetData()->rear_data_array);
  rear_can_ptr_->BREAK_VAR(rear_can_ptr_->GetData()->rear_tof_data_array);

  checking_task_queue_.push(wait_close_res_.task_no_);
  // head UWB
  head_can_ptr_->LINK_VAR(head_can_ptr_->GetData()->head_enable_off_ack);
  head_can_ptr_->LINK_VAR(head_can_ptr_->GetData()->head_tof_enable_off_ack);
  rear_can_ptr_->LINK_VAR(rear_can_ptr_->GetData()->rear_enable_off_ack);
  rear_can_ptr_->LINK_VAR(rear_can_ptr_->GetData()->rear_tof_enable_off_ack);
  head_can_ptr_->Operate("head_enable_off", std::vector<uint8_t>{});
  head_can_ptr_->Operate("head_tof_enable_off", std::vector<uint8_t>{});
  rear_can_ptr_->Operate("rear_enable_off", std::vector<uint8_t>{});
  rear_can_ptr_->Operate("rear_tof_enable_off", std::vector<uint8_t>{});

  INFO("UWB closed successfully");
  std::unique_lock<std::mutex> lock(task_checking_.mt_);
  task_checking_.cv_.notify_one();
  return true;
}

bool UWBCarpo::Initialize()
{
  uint8_t buf[8] = {0};
  uint32_t index = 0;
  initializing_ = true;
  INFO("NOW UWBCarpo::Initialize");
  // get random UWBConnectInfo
  if (use_static_mac_) {
    uwb_connect_info_.session_id = uwb_config_.session_id;
    uwb_connect_info_.controller_mac = uwb_config_.controller_mac;
    uwb_connect_info_.head_tof_mac = uwb_config_.head_tof_mac;
    uwb_connect_info_.head_uwb_mac = uwb_config_.head_uwb_mac;
    uwb_connect_info_.rear_tof_mac = uwb_config_.rear_tof_mac;
    uwb_connect_info_.rear_uwb_mac = uwb_config_.rear_uwb_mac;
  } else {
    uwb_connect_info_.session_id = GenerateRandomNumber(0xFF, 0x7FFFFF00);  //  get  nx sn
    uwb_connect_info_.controller_mac = GenerateRandomNumber(0x00FF, 0x3F00);  // 1-5
    uwb_connect_info_.head_tof_mac = GenerateRandomNumber(0x3F01, 0x6F00);
    uwb_connect_info_.head_uwb_mac = GenerateRandomNumber(0x6F01, 0x9F00);
    uwb_connect_info_.rear_tof_mac = GenerateRandomNumber(0x9F01, 0xCF00);
    uwb_connect_info_.rear_uwb_mac = GenerateRandomNumber(0xCF01, 0xFF00);
  }

  INFO(
    "session_id=%04x mac=%02x, %02x, %02x, %02x, %02x", uwb_connect_info_.session_id,
    uwb_connect_info_.controller_mac,
    uwb_connect_info_.head_tof_mac,
    uwb_connect_info_.head_uwb_mac,
    uwb_connect_info_.rear_tof_mac,
    uwb_connect_info_.rear_uwb_mac);
  // head UWB
  memcpy(&buf[0], &uwb_connect_info_.session_id, sizeof(uwb_connect_info_.session_id));
  memcpy(&buf[4], &uwb_connect_info_.controller_mac, sizeof(uwb_connect_info_.controller_mac));

  auto head_uwb_init_data = std::vector<uint8_t>();
  memcpy(&buf[6], &uwb_connect_info_.head_uwb_mac, sizeof(uwb_connect_info_.head_uwb_mac));

  for (int i = 0; i < 8; i++) {
    head_uwb_init_data.push_back(buf[i]);
  }

  // head TOF UWB
  auto head_tof_init_data = std::vector<uint8_t>();
  memcpy(&buf[6], &uwb_connect_info_.head_tof_mac, sizeof(uwb_connect_info_.head_tof_mac));

  for (int i = 0; i < 8; i++) {
    head_tof_init_data.push_back(buf[i]);
  }
  head_can_ptr_->LINK_VAR(head_can_ptr_->GetData()->head_tof_enable_initial_ack);
  head_can_ptr_->LINK_VAR(head_can_ptr_->GetData()->head_enable_initial_ack);
  head_can_ptr_->Operate("head_tof_enable_initial", head_tof_init_data);
  head_can_ptr_->Operate("head_enable_initial", head_uwb_init_data);

  // rear UWB
  auto rear_uwb_init_data = std::vector<uint8_t>();
  memcpy(&buf[6], &uwb_connect_info_.rear_uwb_mac, sizeof(uwb_connect_info_.rear_uwb_mac));

  for (int i = 0; i < 8; i++) {
    rear_uwb_init_data.push_back(buf[i]);
  }

  // rear TOF UWB
  auto rear_tof_init_data = std::vector<uint8_t>();
  memcpy(&buf[6], &uwb_connect_info_.rear_tof_mac, sizeof(uwb_connect_info_.rear_tof_mac));

  for (int i = 0; i < 8; i++) {
    rear_tof_init_data.push_back(buf[i]);
  }
  rear_can_ptr_->LINK_VAR(rear_can_ptr_->GetData()->rear_tof_enable_initial_ack);
  rear_can_ptr_->LINK_VAR(rear_can_ptr_->GetData()->rear_enable_initial_ack);
  rear_can_ptr_->Operate("rear_tof_enable_initial", rear_tof_init_data);
  rear_can_ptr_->Operate("rear_enable_initial", rear_uwb_init_data);

  std::vector<std::tuple<bool *, std::string, bool, std::vector<uint8_t> *>> checks {
    std::make_tuple(
      &head_enable_initial_, std::string(
        "head_enable_initial"), true, &head_uwb_init_data),
    std::make_tuple(
      &head_tof_enable_initial_, std::string(
        "head_tof_enable_initial"), true, &head_tof_init_data),
    std::make_tuple(
      &rear_enable_initial_, std::string(
        "rear_enable_initial"), false, &rear_uwb_init_data),
    std::make_tuple(
      &rear_tof_enable_initial_, std::string(
        "rear_tof_enable_initial"), false, &rear_tof_init_data)};
  bool check_result = ifFailThenRetry(wait_init_res_, 4, 8000, checks);
  initializing_ = false;
  if (!check_result) {
    ERROR("UWB initialized failed.");
    for (auto & check : checks) {
      if (!*std::get<0>(check)) {
        ERROR_STREAM(std::get<1>(check) << " failed!");
      }
    }
    return false;
  }
  INFO("UWB initialized successfully");
  return true;
}

bool UWBCarpo::GetVersion()
{
  INFO("UWB get version successfully");
  return true;
}

void UWBCarpo::HandleCan0Messages(
  std::string & name,
  std::shared_ptr<cyberdog::device::UWBRearData> data)
{
  INFO_STREAM_MILLSECONDS(2000, "~~~~ can0 uwb callback ~~~~~ ");
  INFO_STREAM_MILLSECONDS(2000, "    name ==   " << name);

  if (name == "rear_enable_initial_ack" || name == "rear_tof_enable_initial_ack") {
    rear_enable_initial_ = true;
    rear_tof_enable_initial_ = true;
    rear_can_ptr_->BREAK_VAR(rear_can_ptr_->GetData()->rear_enable_initial_ack);
    rear_can_ptr_->BREAK_VAR(rear_can_ptr_->GetData()->rear_tof_enable_initial_ack);
  }

  if (rear_enable_initial_ && rear_tof_enable_initial_) {
    if (name == "rear_enable_on_ack" || name == "rear_tof_enable_on_ack") {
      rear_turn_on_ = true;
      rear_tof_turn_on_ = true;
      rear_can_ptr_->BREAK_VAR(rear_can_ptr_->GetData()->rear_enable_on_ack);
      rear_can_ptr_->BREAK_VAR(rear_can_ptr_->GetData()->rear_tof_enable_on_ack);
    }
  }

  if (name == "rear_enable_off_ack" || name == "rear_tof_enable_off_ack") {
    rear_turn_on_ = false;
    rear_tof_turn_on_ = false;
    rear_can_ptr_->BREAK_VAR(rear_can_ptr_->GetData()->rear_enable_off_ack);
    rear_can_ptr_->BREAK_VAR(rear_can_ptr_->GetData()->rear_tof_enable_off_ack);
  }

  {
    std::unique_lock<std::mutex> lock(wait_open_res_.mt_);
    if (!checking_task_queue_.empty() && checking_task_queue_.front() == wait_open_res_.task_no_ &&
      rear_tof_turn_on_ && rear_turn_on_ && head_tof_turn_on_ && head_turn_on_)
    {
      head_can_ptr_->LINK_VAR(head_can_ptr_->GetData()->head_data_array);
      head_can_ptr_->LINK_VAR(head_can_ptr_->GetData()->head_tof_data_array);
      rear_can_ptr_->LINK_VAR(rear_can_ptr_->GetData()->rear_data_array);
      rear_can_ptr_->LINK_VAR(rear_can_ptr_->GetData()->rear_tof_data_array);
      wait_open_res_.cv_.notify_one();
      INFO("can0 notify_one");
    }
  }

  {
    std::unique_lock<std::mutex> lock(wait_close_res_.mt_);
    if (!checking_task_queue_.empty() && checking_task_queue_.front() == wait_close_res_.task_no_ &&
      !rear_tof_turn_on_ && !rear_turn_on_ && !head_tof_turn_on_ && !head_turn_on_)
    {
      wait_close_res_.cv_.notify_one();
      INFO("can0 notify_one");
    }
  }

  {
    std::unique_lock<std::mutex> lock(wait_init_res_.mt_);
    if (initializing_ && rear_tof_enable_initial_ && rear_enable_initial_ &&
      head_tof_enable_initial_ && head_enable_initial_)
    {
      wait_init_res_.cv_.notify_one();
      INFO("can0 notify_one");
    }
  }

  if (rear_tof_turn_on_ && head_turn_on_) {
    if (name == "rear_data_array" || name == "rear_tof_data_array") {
      INFO_MILLSECONDS(5000, "Receive rear data or rear tof data");

      // UWB data
      float dist = data->rear_data_array[0] + (data->rear_data_array[1] << 8);
      short angle = data->rear_data_array[2] + (data->rear_data_array[3] << 8);  // NOLINT
      float nLos = data->rear_data_array[4];
      short rssi_1 = data->rear_data_array[6] + (data->rear_data_array[7] << 8);  // NOLINT
      short rssi_2 = data->rear_data_array[8] + (data->rear_data_array[9] << 8);  // NOLINT

      int index = static_cast<int>(Type::RearUWB);
      // ros_uwb_status_.data[1].position = LeftPose(dist, static_cast<float>(angle));
      std::unique_lock<std::shared_mutex> write_lock0(raw_data_mutex_0_);
      ros_uwb_status_.data[index].dist = dist / 100;
      ros_uwb_status_.data[index].angle = DegToRad(format_9_7(angle));
      ros_uwb_status_.data[index].n_los = nLos;
      ros_uwb_status_.data[index].rssi_1 = format_8_8(rssi_1);
      ros_uwb_status_.data[index].rssi_2 = format_8_8(rssi_2);

      // TOF data
      float dist_tof = data->rear_tof_data_array[0] + (data->rear_tof_data_array[1] << 8); // NOLINT
      short angle_tof = data->rear_tof_data_array[2] + (data->rear_tof_data_array[3] << 8);   // NOLINT
      float nLos_tof = data->rear_tof_data_array[4];
      float rssi_1_tof = data->rear_tof_data_array[6] + (data->rear_tof_data_array[7] << 8);  // NOLINT
      short rssi_2_tof = data->rear_tof_data_array[8] + (data->rear_tof_data_array[9] << 8);  // NOLINT

      // ros_uwb_status_.data[2].position = RightPose(dist_tof, angle_tof);
      index = static_cast<int>(Type::RearTOF);
      ros_uwb_status_.data[index].dist = dist_tof / 100;
      ros_uwb_status_.data[index].angle = DegToRad(format_9_7(angle_tof));
      ros_uwb_status_.data[index].n_los = nLos_tof;
      ros_uwb_status_.data[index].rssi_1 = format_8_8(rssi_1_tof);
      ros_uwb_status_.data[index].rssi_2 = format_8_8(rssi_2_tof);

      raw_data_updated[0] = true;

      INFO_MILLSECONDS(5000, "--------------------[UWB]----------------------");
      INFO_MILLSECONDS(5000, "%02X, %02X", data->rear_data_array[0], data->rear_data_array[1]);
      INFO_MILLSECONDS(5000, "Current dist : %f", dist);
      INFO_MILLSECONDS(5000, "Current angle : %f", format_9_7(angle));
      INFO_MILLSECONDS(5000, "Current nLos : %f", nLos);
      INFO_MILLSECONDS(5000, "Current rssi_1 : %f", format_8_8(rssi_1));
      INFO_MILLSECONDS(5000, "Current rssi_2 : %f", format_8_8(rssi_2));

      INFO_MILLSECONDS(5000, "--------------------[TOF]----------------------");
      INFO_MILLSECONDS(
        5000, "%02X, %02X", data->rear_tof_data_array[0],
        data->rear_tof_data_array[1]);
      INFO_MILLSECONDS(5000, "Current dist : %f", dist_tof);
      INFO_MILLSECONDS(5000, "Current angle : %f", format_9_7(angle_tof));
      INFO_MILLSECONDS(5000, "Current nLos : %f", nLos_tof);
      INFO_MILLSECONDS(5000, "Current rssi_1 : %f", format_8_8(rssi_1_tof));
      INFO_MILLSECONDS(5000, "Current rssi_2 : %f", format_8_8(rssi_2_tof));
    }
  }
}

void UWBCarpo::HandleCan1Messages(
  std::string & name,
  std::shared_ptr<cyberdog::device::UWBHeadData> data)
{
  INFO_STREAM_MILLSECONDS(2000, "~~~~ can1 uwb callback ~~~~~ ");
  INFO_STREAM_MILLSECONDS(2000, "    name ==   " << name);

  if (name == "head_enable_initial_ack" || name == "head_tof_enable_initial_ack") {
    head_enable_initial_ = true;
    head_tof_enable_initial_ = true;
    head_can_ptr_->BREAK_VAR(head_can_ptr_->GetData()->head_enable_initial_ack);
    head_can_ptr_->BREAK_VAR(head_can_ptr_->GetData()->head_tof_enable_initial_ack);
  }

  if (head_enable_initial_ && head_tof_enable_initial_) {
    if (name == "head_enable_on_ack" || name == "head_tof_enable_on_ack") {
      head_turn_on_ = true;
      head_tof_turn_on_ = true;
      head_can_ptr_->BREAK_VAR(head_can_ptr_->GetData()->head_enable_on_ack);
      head_can_ptr_->BREAK_VAR(head_can_ptr_->GetData()->head_tof_enable_on_ack);
    }
  }

  if (name == "head_enable_off_ack" || name == "head_tof_enable_off_ack") {
    head_turn_on_ = false;
    head_tof_turn_on_ = false;
    head_can_ptr_->BREAK_VAR(head_can_ptr_->GetData()->head_enable_off_ack);
    head_can_ptr_->BREAK_VAR(head_can_ptr_->GetData()->head_tof_enable_off_ack);
  }

  {
    std::unique_lock<std::mutex> lock(wait_open_res_.mt_);
    if (!checking_task_queue_.empty() && checking_task_queue_.front() == wait_open_res_.task_no_ &&
      rear_tof_turn_on_ && rear_turn_on_ && head_tof_turn_on_ && head_turn_on_)
    {
      head_can_ptr_->LINK_VAR(head_can_ptr_->GetData()->head_data_array);
      head_can_ptr_->LINK_VAR(head_can_ptr_->GetData()->head_tof_data_array);
      rear_can_ptr_->LINK_VAR(rear_can_ptr_->GetData()->rear_data_array);
      rear_can_ptr_->LINK_VAR(rear_can_ptr_->GetData()->rear_tof_data_array);
      wait_open_res_.cv_.notify_one();
      INFO("can1 notify_one");
    }
  }

  {
    std::unique_lock<std::mutex> lock(wait_close_res_.mt_);
    if (!checking_task_queue_.empty() && checking_task_queue_.front() == wait_close_res_.task_no_ &&
      !rear_tof_turn_on_ && !rear_turn_on_ && !head_tof_turn_on_ && !head_turn_on_)
    {
      wait_close_res_.cv_.notify_one();
      INFO("can1 notify_one");
    }
  }

  {
    std::unique_lock<std::mutex> lock(wait_init_res_.mt_);
    if (initializing_ && rear_tof_enable_initial_ && rear_enable_initial_ &&
      head_tof_enable_initial_ && head_enable_initial_)
    {
      wait_init_res_.cv_.notify_one();
      INFO("can1 notify_one");
    }
  }

  if (head_tof_turn_on_ && head_turn_on_) {
    if (name == "head_data_array" || name == "head_tof_data_array") {
      INFO_MILLSECONDS(5000, "Receive head data or head tof data.");

      // UWB data
      float dist = data->head_data_array[0] + (data->head_data_array[1] << 8);
      short angle = data->head_data_array[2] + (data->head_data_array[3] << 8);   // NOLINT
      float nLos = data->head_data_array[4];
      short rssi_1 = data->head_data_array[6] + (data->head_data_array[7] << 8);  // NOLINT
      short rssi_2 = data->head_data_array[8] + (data->head_data_array[9] << 8);  // NOLINT

      int index = static_cast<int>(Type::HeadUWB);
      // ros_uwb_status_.data[0].position = FrontPose(dist, angle);
      std::unique_lock<std::shared_mutex> write_lock1(raw_data_mutex_1_);
      ros_uwb_status_.data[index].dist = dist / 100;
      ros_uwb_status_.data[index].angle = DegToRad(format_9_7(angle));
      ros_uwb_status_.data[index].n_los = nLos;
      ros_uwb_status_.data[index].rssi_1 = format_8_8(rssi_1);
      ros_uwb_status_.data[index].rssi_2 = format_8_8(rssi_2);

      // TOF data
      float dist_tof = data->head_tof_data_array[0] + (data->head_tof_data_array[1] << 8);
      short angle_tof = data->head_tof_data_array[2] + (data->head_tof_data_array[3] << 8);   // NOLINT
      float nLos_tof = data->head_tof_data_array[4];
      short rssi_1_tof = data->head_tof_data_array[6] + (data->head_tof_data_array[7] << 8);  // NOLINT
      short rssi_2_tof = data->head_tof_data_array[8] + (data->head_tof_data_array[9] << 8);  // NOLINT

      // ros_uwb_status_.data[3].position = BackPose(dist_tof, angle_tof);
      index = static_cast<int>(Type::HeadTOF);
      ros_uwb_status_.data[index].dist = dist_tof / 100;
      ros_uwb_status_.data[index].angle = DegToRad(format_9_7(angle_tof));
      ros_uwb_status_.data[index].n_los = nLos_tof;
      ros_uwb_status_.data[index].rssi_1 = format_8_8(rssi_1_tof);
      ros_uwb_status_.data[index].rssi_2 = format_8_8(rssi_2_tof);

      raw_data_updated[1] = true;

      INFO_MILLSECONDS(5000, "--------------------[UWB]----------------------");
      INFO_MILLSECONDS(5000, "%02X, %02X", data->head_data_array[0], data->head_data_array[1]);
      INFO_MILLSECONDS(5000, "Current dist : %f", dist);
      INFO_MILLSECONDS(5000, "Current angle : %f", format_9_7(angle));
      INFO_MILLSECONDS(5000, "Current nLos : %f", nLos);
      INFO_MILLSECONDS(5000, "Current rssi_1 : %f", format_8_8(rssi_1));
      INFO_MILLSECONDS(5000, "Current rssi_2 : %f", format_8_8(rssi_2));

      INFO_MILLSECONDS(5000, "--------------------[TOF]----------------------");
      INFO_MILLSECONDS(
        3000, "%02X, %02X", data->head_tof_data_array[0],
        data->head_tof_data_array[1]);
      INFO_MILLSECONDS(5000, "Current dist : %f", dist_tof);
      INFO_MILLSECONDS(5000, "Current angle : %f", format_9_7(angle_tof));
      INFO_MILLSECONDS(5000, "Current nLos : %f", nLos_tof);
      INFO_MILLSECONDS(5000, "Current rssi_1 : %f", format_8_8(rssi_1_tof));
      INFO_MILLSECONDS(5000, "Current rssi_2 : %f", format_8_8(rssi_2_tof));
    }
  }
}

void UWBCarpo::RunTask()
{
  while (threading_) {
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
    bool updated = UwbRawStatusMsg2Ros();
    INFO_STREAM_MILLSECONDS(
      2000,
      "activated_=" << activated_ << " queue_.empty()=" << queue_.empty() << " updated=" <<
        updated);
    if (activated_ && updated) {
      if (queue_.empty()) {
        continue;
      }

      while (queue_.size() > 5) {
        queue_.pop_front();
      }

      // get msg from queue
      auto msg = queue_.back();

      // publish msgs
      INFO_MILLSECONDS(5000, "Publish uwb raw mags.");
      status_function_(msg);
    }
  }
}


bool UWBCarpo::InitializeCanCommunication()
{
  // Config the battery file
  auto local_share_dir = ament_index_cpp::get_package_share_directory("params");
  auto head_path = local_share_dir + std::string("/toml_config/device/uwb_head.toml");
  auto rear_path = local_share_dir + std::string("/toml_config/device/uwb_rear.toml");

  // Create Protocol for `BMSStatus` data
  head_can_ptr_ = std::make_shared<cyberdog::embed::Protocol<UWBHeadData>>(head_path, false);
  rear_can_ptr_ = std::make_shared<cyberdog::embed::Protocol<UWBRearData>>(rear_path, false);

  if (head_can_ptr_ == nullptr || rear_can_ptr_ == nullptr) {
    INFO("Create UWB Can bridge error.");
    return false;
  }

  rear_can_ptr_->SetDataCallback(
    std::bind(
      &UWBCarpo::HandleCan0Messages,
      this, std::placeholders::_1, std::placeholders::_2));

  head_can_ptr_->SetDataCallback(
    std::bind(
      &UWBCarpo::HandleCan1Messages,
      this, std::placeholders::_1, std::placeholders::_2));
  return true;
}

void UWBCarpo::RunSimulation()
{
  std::thread simulation_task([this]()
    {
      while (simulation_) {
        ros_uwb_status_.data.clear();
        // header
        struct timespec ts;
        clock_gettime(CLOCK_REALTIME, &ts);

        // p1
        // 1 head 2 rear 3 rear TOF  4 head TOF
        // p2
        // 1 head TOF  2 rear 3 rear TOF  head

        constexpr int NumberUWB = 4;
        for (int i = 0; i < NumberUWB; i++) {
          UwbSignleStatusMsg uwb;
          uwb.header.frame_id = "uwb_sensor";
          uwb.header.stamp.nanosec = ts.tv_nsec;
          uwb.header.stamp.sec = ts.tv_sec;
          uwb.dist = GenerateRandomNumber(0, 100);
          uwb.angle = GenerateRandomNumber(0, 100);
          uwb.n_los = GenerateRandomNumber(0, 100);
          uwb.rssi_1 = GenerateRandomNumber(0, 100);
          uwb.rssi_1 = GenerateRandomNumber(0, 100);
          ros_uwb_status_.data.push_back(uwb);
        }

        std::this_thread::sleep_for(std::chrono::seconds(1));
      }
    });
  simulation_task.detach();
}

int UWBCarpo::GenerateRandomNumber(int start, int end)
{
  std::random_device rd;   // 将用于为随机数引擎获得种子
  std::mt19937 gen(rd());  // 以播种标准 mersenne_twister_engine
  std::uniform_int_distribution<> dis(start, end);  // [start end]
  return dis(gen);
}

protocol::msg::UwbArray UWBCarpo::ToROS()
{
  std::lock_guard<std::mutex> lock(mutex_);
  return ros_uwb_status_;
}

int UWBCarpo::ConvertAngle(const int & angle)
{
  return 0;
}

int UWBCarpo::ConvertRange(const int & rangle)
{
  return 0;
}

std::vector<float> UWBCarpo::FrontPose(const float & dist, const float & angle)
{
  constexpr float offset_x = 17.0f;
  constexpr float offset_y = 0.0f;
  constexpr float offset_z = 16.4f;

  return std::vector<float> {
    (dist * std::cos(DegToRad(-angle)) + offset_x) / 100,
    (dist * std::sin(DegToRad(-angle)) + offset_y) / 100,
    (0.0f + offset_z) / 100
  };
}

std::vector<float> UWBCarpo::BackPose(const float & dist, const float & angle)
{
  constexpr float offset_x = 21.85f;
  constexpr float offset_y = 0.0f;
  constexpr float offset_z = -0.495f;

  return std::vector<float> {
    (dist * std::cos(DegToRad(180.0 - angle)) + offset_x) / 100,
    (dist * std::sin(DegToRad(180.0 - angle)) + offset_y) / 100,
    (0.0f + offset_z) / 100
  };
}

std::vector<float> UWBCarpo::RightPose(const float & dist, const float & angle)
{
  constexpr float offset_x = -2.3f;
  constexpr float offset_y = 8.45f;
  constexpr float offset_z = -0.325f;
  // INFO("----------------------------------------------------------------------------");
  // INFO("angle = %f, dist = %f", angle, dist);
  // INFO("----------------------------------------------------------------------------");
  return std::vector<float> {
    (dist * std::cos(DegToRad(-90.0 - angle)) + offset_x) / 100,
    (dist * std::sin(DegToRad(-90.0 - angle)) + offset_y) / 100,
    (0.0f + offset_z) / 100
  };
}


std::vector<float> UWBCarpo::LeftPose(const float & dist, const float & angle)
{
  constexpr float offset_x = -2.35f;
  constexpr float offset_y = -8.45f;
  constexpr float offset_z = -0.325f;

  return std::vector<float> {
    (dist * std::cos(DegToRad(90.0 - angle)) + offset_x) / 100,
    (dist * std::sin(DegToRad(90.0 - angle)) + offset_y) / 100,
    (0.0f + offset_z) / 100
  };
}

bool UWBCarpo::LoadUWBTomlConfig()
{
  auto local_share_dir = ament_index_cpp::get_package_share_directory("params");
  auto path = local_share_dir + std::string("/toml_config/device/uwb_config.toml");

  if (!cyberdog::common::CyberdogToml::ParseFile(path.c_str(), params_toml_)) {
    ERROR("Params config file is not in toml format");
    return false;
  }

  uwb_config_.front_back_threshold = toml::find<double>(params_toml_, "front_back_threshold");
  uwb_config_.left_right_threshold = toml::find<double>(params_toml_, "left_right_threshold");

  // enable
  use_uwb_ = toml::find<bool>(params_toml_, "enable");

  // HeadUWB
  uwb_config_.AoA_F_NMAX = toml::find<double>(params_toml_, "HeadUWB", "AoA_F_NMAX");
  uwb_config_.AoA_F_PMAX = toml::find<double>(params_toml_, "HeadUWB", "AoA_F_PMAX");
  uwb_config_.AoA_B_NMAX = toml::find<double>(params_toml_, "HeadUWB", "AoA_B_NMAX");
  uwb_config_.AoA_B_PMAX = toml::find<double>(params_toml_, "HeadUWB", "AoA_B_PMAX");

  // RearUWB
  uwb_config_.AoA_L_NMAX = toml::find<double>(params_toml_, "RearUWB", "AoA_L_NMAX");
  uwb_config_.AoA_L_PMAX = toml::find<double>(params_toml_, "RearUWB", "AoA_L_PMAX");
  uwb_config_.AoA_R_NMAX = toml::find<double>(params_toml_, "RearUWB", "AoA_R_NMAX");
  uwb_config_.AoA_R_PMAX = toml::find<double>(params_toml_, "RearUWB", "AoA_R_PMAX");
  // uwb connect info
  use_static_mac_ = toml::find<bool>(params_toml_, "use_static_mac");
  uwb_config_.session_id = toml::find<uint32_t>(params_toml_, "UWBConnectionInfo", "session_id");
  uwb_config_.controller_mac = toml::find<uint16_t>(
    params_toml_, "UWBConnectionInfo",
    "controller_mac");
  uwb_config_.head_tof_mac =
    toml::find<uint16_t>(params_toml_, "UWBConnectionInfo", "head_tof_mac");
  uwb_config_.head_uwb_mac =
    toml::find<uint16_t>(params_toml_, "UWBConnectionInfo", "head_uwb_mac");
  uwb_config_.rear_tof_mac =
    toml::find<uint16_t>(params_toml_, "UWBConnectionInfo", "rear_tof_mac");
  uwb_config_.rear_uwb_mac =
    toml::find<uint16_t>(params_toml_, "UWBConnectionInfo", "rear_uwb_mac");

  INFO("get uwb_config.toml end");

  return true;
}

UWBConfig & UWBCarpo::GetUWBConfig()
{
  return uwb_config_;
}


void UWBCarpo::SetData(const Type & type, const UwbSignleStatusMsg & data)
{
  UwbSignleStatusMsg uwb;
  switch (type) {
    case Type::HeadTOF:
      {
        ros_uwb_status_.data[3] = uwb;
      }
      break;

    case Type::HeadUWB:
      {
        ros_uwb_status_.data[3] = uwb;
      }
      break;

    case Type::RearTOF:
      {
        ros_uwb_status_.data[3] = uwb;
      }
      break;

    case Type::RearUWB:
      {
        ros_uwb_status_.data[3] = uwb;
      }
      break;

    default:
      break;
  }
}

void UWBCarpo::Debug2String(
  const Type & type,
  const geometry_msgs::msg::PoseStamped & uwb_posestamped)
{
  uint8_t flag = 0;
  // ros_uwb_status_.data[0] // head uwb front
  // ros_uwb_status_.data[3] // head tof back

  // ros_uwb_status_.data[1] // rear uwb left
  // ros_uwb_status_.data[2] // rear tof right

  // float32 dist
  // float32 angle
  // uint64 n_los
  // uint16 rssi_1
  // uint16 rssi_2

  switch (type) {
    case Type::HeadTOF:
      INFO("################## [Current uwb type : Type::HeadTOF] ##################");
      break;
    case Type::HeadUWB:
      INFO("################## [Current uwb type : Type::HeadUWB] ##################");
      break;
    case Type::RearTOF:
      INFO("################## [Current uwb type : Type::RearTOF] ##################");
      break;
    case Type::RearUWB:
      INFO("################## [Current uwb type : Type::RearUWB] ##################");
      break;
    default:
      flag = 1;
      break;
  }
  if (flag == 0) {
    INFO("Current dist : %f", ros_uwb_status_.data[static_cast<int>(type)].dist);
    INFO("Current angle : %f", ros_uwb_status_.data[static_cast<int>(type)].angle);
    INFO("Current nLos : %f", ros_uwb_status_.data[static_cast<int>(type)].n_los);
    INFO("Current rssi_1 : %f", ros_uwb_status_.data[static_cast<int>(type)].rssi_1);
    INFO("Current rssi_2 : %f", ros_uwb_status_.data[static_cast<int>(type)].rssi_2);
    INFO("position x = %f", uwb_posestamped.pose.position.x);
    INFO("position y = %f", uwb_posestamped.pose.position.y);
    INFO("position z = %f", uwb_posestamped.pose.position.z);
  }
}

void UWBCarpo::Debug2String(const UwbSignleStatusMsg & uwb_msg)
{
  INFO_MILLSECONDS(3000, "########################################");
  INFO_MILLSECONDS(3000, "frame id = %s", uwb_msg.header.frame_id.c_str());
  INFO_MILLSECONDS(3000, "dist = %f", uwb_msg.dist);
  INFO_MILLSECONDS(3000, "angle = %f", uwb_msg.angle);
  INFO_MILLSECONDS(3000, "n_los = %f", uwb_msg.n_los);
  INFO_MILLSECONDS(3000, "rssi_1 = %f", uwb_msg.rssi_1);
  INFO_MILLSECONDS(3000, "rssi_2 = %f", uwb_msg.rssi_2);
}

bool UWBCarpo::UwbRawStatusMsg2Ros()
{
  // ros_uwb_status_.data[0] // head uwb front
  // ros_uwb_status_.data[3] // head tof back

  // ros_uwb_status_.data[1] // rear uwb left
  // ros_uwb_status_.data[2] // rear tof right

  UwbSignleStatusMsg uwb_posestamped;
  bool debug_to_string = false;
  constexpr double front_back_threshold = 7.0;
  constexpr auto eps = std::numeric_limits<float>::epsilon();

  std::shared_lock<std::shared_mutex> read_lock0(raw_data_mutex_0_);
  std::shared_lock<std::shared_mutex> read_lock1(raw_data_mutex_1_);
  if (!(raw_data_updated[0] && raw_data_updated[1])) {
    return false;
  } else {
    raw_data_updated[0] = false;
    raw_data_updated[1] = false;
    if (queue_.size() > 5) {
      queue_.pop_front();
    }
  }
  auto & ros_uwb_status_front = ros_uwb_status_.data[static_cast<int>(Type::HeadTOF)];
  auto & ros_uwb_status_back = ros_uwb_status_.data[static_cast<int>(Type::HeadUWB)];
  auto & ros_uwb_status_left = ros_uwb_status_.data[static_cast<int>(Type::RearUWB)];
  auto & ros_uwb_status_right = ros_uwb_status_.data[static_cast<int>(Type::RearTOF)];


  double delta = std::fabs(ros_uwb_status_front.rssi_1 - ros_uwb_status_back.rssi_1);

  if (delta > front_back_threshold) {
    constexpr double AoA_F_NMAX = -48.0f;
    constexpr double AoA_F_PMAX = 60.0f;

    if (ros_uwb_status_front.rssi_1 > ros_uwb_status_back.rssi_1) {
      // Head UWB
      if (ros_uwb_status_front.angle > AoA_F_NMAX &&
        ros_uwb_status_front.angle < AoA_F_PMAX) // NOLINT
      {
        struct timespec time_stu;
        clock_gettime(CLOCK_REALTIME, &time_stu);
        uwb_posestamped.header.stamp.nanosec = time_stu.tv_nsec;
        uwb_posestamped.header.stamp.sec = time_stu.tv_sec;
        uwb_posestamped.header.frame_id = "head_tof";
        uwb_posestamped.dist = ros_uwb_status_front.dist;
        uwb_posestamped.angle = ros_uwb_status_front.angle;
        uwb_posestamped.n_los = ros_uwb_status_front.n_los;
        uwb_posestamped.rssi_1 = ros_uwb_status_front.rssi_1;
        uwb_posestamped.rssi_2 = ros_uwb_status_front.rssi_2;
        queue_.push_back(uwb_posestamped);
        debug_to_string = true;
      } else if (ros_uwb_status_front.angle > AoA_F_PMAX ||  // NOLINT
        helper_functions::rel_eq<double>(ros_uwb_status_front.angle, AoA_F_PMAX, eps))
      {
        // Rear TOF
        struct timespec time_stu;
        clock_gettime(CLOCK_REALTIME, &time_stu);
        uwb_posestamped.header.stamp.nanosec = time_stu.tv_nsec;
        uwb_posestamped.header.stamp.sec = time_stu.tv_sec;
        uwb_posestamped.header.frame_id = "rear_tof";
        uwb_posestamped.dist = ros_uwb_status_right.dist;
        uwb_posestamped.angle = ros_uwb_status_right.angle;
        uwb_posestamped.n_los = ros_uwb_status_right.n_los;
        uwb_posestamped.rssi_1 = ros_uwb_status_right.rssi_1;
        uwb_posestamped.rssi_2 = ros_uwb_status_right.rssi_2;
        queue_.push_back(uwb_posestamped);
        debug_to_string = true;
      } else if (ros_uwb_status_front.angle < AoA_F_NMAX ||  // NOLINT
        helper_functions::rel_eq<double>(ros_uwb_status_front.angle, AoA_F_NMAX, eps))
      {
        // Rear UWB
        struct timespec time_stu;
        clock_gettime(CLOCK_REALTIME, &time_stu);
        uwb_posestamped.header.stamp.nanosec = time_stu.tv_nsec;
        uwb_posestamped.header.stamp.sec = time_stu.tv_sec;
        uwb_posestamped.header.frame_id = "rear_uwb";
        uwb_posestamped.dist = ros_uwb_status_left.dist;
        uwb_posestamped.angle = ros_uwb_status_left.angle;
        uwb_posestamped.n_los = ros_uwb_status_left.n_los;
        uwb_posestamped.rssi_1 = ros_uwb_status_left.rssi_1;
        uwb_posestamped.rssi_2 = ros_uwb_status_left.rssi_2;
        queue_.push_back(uwb_posestamped);
        debug_to_string = true;
      }
    } else {
      constexpr double AoA_B_NMAX = -60.0f;
      constexpr double AoA_B_PMAX = 60.0f;

      if (ros_uwb_status_back.angle > AoA_B_NMAX &&
        ros_uwb_status_back.angle < AoA_B_PMAX)
      {
        // Head TOF
        struct timespec time_stu;
        clock_gettime(CLOCK_REALTIME, &time_stu);
        uwb_posestamped.header.stamp.nanosec = time_stu.tv_nsec;
        uwb_posestamped.header.stamp.sec = time_stu.tv_sec;
        uwb_posestamped.header.frame_id = "head_uwb";
        uwb_posestamped.dist = ros_uwb_status_back.dist;
        uwb_posestamped.angle = ros_uwb_status_back.angle;
        uwb_posestamped.n_los = ros_uwb_status_back.n_los;
        uwb_posestamped.rssi_1 = ros_uwb_status_back.rssi_1;
        uwb_posestamped.rssi_2 = ros_uwb_status_back.rssi_2;
        queue_.push_back(uwb_posestamped);
        debug_to_string = true;
      } else if (ros_uwb_status_back.angle > AoA_B_PMAX ||  // NOLINT
        helper_functions::rel_eq<double>(ros_uwb_status_back.angle, AoA_B_PMAX, eps))    // NOLINT
      {
        // Rear UWB
        struct timespec time_stu;
        clock_gettime(CLOCK_REALTIME, &time_stu);
        uwb_posestamped.header.stamp.nanosec = time_stu.tv_nsec;
        uwb_posestamped.header.stamp.sec = time_stu.tv_sec;
        uwb_posestamped.header.frame_id = "rear_uwb";
        uwb_posestamped.dist = ros_uwb_status_left.dist;
        uwb_posestamped.angle = ros_uwb_status_left.angle;
        uwb_posestamped.n_los = ros_uwb_status_left.n_los;
        uwb_posestamped.rssi_1 = ros_uwb_status_left.rssi_1;
        uwb_posestamped.rssi_2 = ros_uwb_status_left.rssi_2;
        queue_.push_back(uwb_posestamped);
        debug_to_string = true;
      } else if (ros_uwb_status_back.angle < AoA_B_NMAX ||  // NOLINT
        helper_functions::rel_eq<double>(ros_uwb_status_back.angle, AoA_B_NMAX, eps))    // NOLINT
      {
        // Rear TOF
        struct timespec time_stu;
        clock_gettime(CLOCK_REALTIME, &time_stu);
        uwb_posestamped.header.stamp.nanosec = time_stu.tv_nsec;
        uwb_posestamped.header.stamp.sec = time_stu.tv_sec;
        uwb_posestamped.header.frame_id = "rear_tof";
        uwb_posestamped.dist = ros_uwb_status_right.dist;
        uwb_posestamped.angle = ros_uwb_status_right.angle;
        uwb_posestamped.n_los = ros_uwb_status_right.n_los;
        uwb_posestamped.rssi_1 = ros_uwb_status_right.rssi_1;
        uwb_posestamped.rssi_2 = ros_uwb_status_right.rssi_2;
        queue_.push_back(uwb_posestamped);
        debug_to_string = true;
      }
    }
  } else {
    constexpr double left_right_threshold = 7.0;
    double delta = std::fabs(ros_uwb_status_left.rssi_1 - ros_uwb_status_right.rssi_1);

    if (delta > left_right_threshold) {
      // rear uwb
      if (ros_uwb_status_left.rssi_1 > ros_uwb_status_right.rssi_1) {
        struct timespec time_stu;
        clock_gettime(CLOCK_REALTIME, &time_stu);
        uwb_posestamped.header.stamp.nanosec = time_stu.tv_nsec;
        uwb_posestamped.header.stamp.sec = time_stu.tv_sec;
        uwb_posestamped.header.frame_id = "rear_uwb";
        uwb_posestamped.dist = ros_uwb_status_left.dist;
        uwb_posestamped.angle = ros_uwb_status_left.angle;
        uwb_posestamped.n_los = ros_uwb_status_left.n_los;
        uwb_posestamped.rssi_1 = ros_uwb_status_left.rssi_1;
        uwb_posestamped.rssi_2 = ros_uwb_status_left.rssi_2;
        queue_.push_back(uwb_posestamped);
        debug_to_string = true;
      } else {
        // rear tof
        struct timespec time_stu;
        clock_gettime(CLOCK_REALTIME, &time_stu);
        uwb_posestamped.header.stamp.nanosec = time_stu.tv_nsec;
        uwb_posestamped.header.stamp.sec = time_stu.tv_sec;
        uwb_posestamped.header.frame_id = "rear_tof";
        uwb_posestamped.dist = ros_uwb_status_right.dist;
        uwb_posestamped.angle = ros_uwb_status_right.angle;
        uwb_posestamped.n_los = ros_uwb_status_right.n_los;
        uwb_posestamped.rssi_1 = ros_uwb_status_right.rssi_1;
        uwb_posestamped.rssi_2 = ros_uwb_status_right.rssi_2;
        queue_.push_back(uwb_posestamped);
        debug_to_string = true;
      }
    }
  }

  if (debug_to_string) {
    Debug2String(uwb_posestamped);
  }
  return true;
}

bool UWBCarpo::ifFailThenRetry(
  WaitingForResponse & wfr, int trial_times, int64_t millisec,
  const std::vector<std::tuple<bool *, std::string, bool, std::vector<uint8_t> *>> & checks,
  bool negative)
{
  std::unique_lock<std::mutex> lock(wfr.mt_);
  for (int i = 0; i < trial_times; ++i) {
    INFO("Start wait for timeout!");
    bool is_on = wait_open_res_.cv_.wait_for(
      lock,
      std::chrono::milliseconds(millisec),
      [&]() {
        for (auto & check : checks) {
          if (negative ? *std::get<0>(check) : !*std::get<0>(check)) {
            return false;
          }
        }
        return true;
      });
    if (is_on) {
      return true;
    }
    if (i == trial_times - 1) {
      ERROR("Failed to request!");
      return false;
    }
    for (auto & check : checks) {
      if (negative ? *std::get<0>(check) : !*std::get<0>(check)) {
        if (std::get<2>(check)) {
          head_can_ptr_->Operate(std::get<1>(check), *std::get<3>(check));
        } else {
          rear_can_ptr_->Operate(std::get<1>(check), *std::get<3>(check));
        }
        WARN_STREAM("Retrying " << std::get<1>(check) << " for " << i << " times");
      }
    }
  }
  return true;
}

void UWBCarpo::checkResponse()
{
  while (threading_) {
    std::unique_lock<std::mutex> lock(task_checking_.mt_);
    task_checking_.cv_.wait(lock, [&]() {return !(checking_task_queue_.empty() && threading_);});
    if (!threading_) {
      break;
    }
    if (checking_task_queue_.front() == wait_open_res_.task_no_) {
      lock.unlock();
      std::vector<uint8_t> empty_vector;
      std::vector<std::tuple<bool *, std::string, bool, std::vector<uint8_t> *>> open_checks {
        std::make_tuple(
          &head_turn_on_, std::string(
            "head_enable_on"), true, &empty_vector),
        std::make_tuple(
          &head_tof_turn_on_, std::string(
            "head_tof_enable_on"), true, &empty_vector),
        std::make_tuple(
          &rear_turn_on_, std::string(
            "rear_enable_on"), false, &empty_vector),
        std::make_tuple(
          &rear_tof_turn_on_, std::string(
            "rear_tof_enable_on"), false, &empty_vector)};
      bool result = ifFailThenRetry(wait_open_res_, 3, 2000, open_checks);
      checking_task_queue_.pop();
      if (result) {
        INFO("All UWB device has been linked.");
      } else {
        for (auto & check : open_checks) {
          if (!*std::get<0>(check)) {
            ERROR_STREAM(std::get<1>(check) << " failed!");
          }
        }
      }
    } else if (checking_task_queue_.front() == wait_close_res_.task_no_) {
      lock.unlock();
      std::vector<uint8_t> empty_vector;
      std::vector<std::tuple<bool *, std::string, bool, std::vector<uint8_t> *>> close_checks {
        std::make_tuple(
          &head_turn_on_, std::string(
            "head_enable_off"), true, &empty_vector),
        std::make_tuple(
          &head_tof_turn_on_, std::string(
            "head_tof_enable_off"), true, &empty_vector),
        std::make_tuple(
          &rear_turn_on_, std::string(
            "rear_enable_off"), false, &empty_vector),
        std::make_tuple(
          &rear_tof_turn_on_, std::string(
            "rear_tof_enable_off"), false, &empty_vector)};
      bool result = ifFailThenRetry(wait_close_res_, 3, 2000, close_checks, true);
      checking_task_queue_.pop();
      if (result) {
        INFO("All UWB device has been disconnected.");
      } else {
        for (auto & check : close_checks) {
          if (*std::get<0>(check)) {
            ERROR_STREAM(std::get<1>(check) << " failed!");
          }
        }
      }
    }
  }
}

}  //  namespace device
}  //  namespace cyberdog

PLUGINLIB_EXPORT_CLASS(cyberdog::device::UWBCarpo, cyberdog::device::UWBBase)
