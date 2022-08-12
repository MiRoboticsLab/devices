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

#include "cyberdog_uwb/uwb_plugin.hpp"
#include "cyberdog_uwb/float_comparisons.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "ament_index_cpp/get_package_share_directory.hpp"

namespace cyberdog
{
namespace device
{

UWBCarpo::UWBCarpo()
{
  ros_uwb_status_.data.resize(4);
  if (!LoadUWBTomlConfig()) {
    ERROR("Load UWB parameters error.");
  }
}

bool UWBCarpo::Config()
{
  return true;
}

bool UWBCarpo::Init(
  std::function<void(geometry_msgs::msg::PoseStamped)>
  function_callback, bool simulation)
{
  RegisterTopic(function_callback);
  uwb_thread_ = std::make_shared<std::thread>(std::bind(&UWBCarpo::RunTask, this));
  uwb_thread_->detach();

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

    // Uwb bringup time
    sleep(3);

    // Open uwb
    initialized_finished_ = Open();
    if (!initialized_finished_) {
      INFO("Open uwb open device failed.");
      return initialized_finished_;
    }
  }

  INFO("[UWBCarpo]: %s", "UWBCarpo initialize success.");
  return initialized_finished_;
}

bool UWBCarpo::SelfCheck()
{
  return true;
}

bool UWBCarpo::RegisterTopic(std::function<void(geometry_msgs::msg::PoseStamped)> function_callback)
{
  status_function_ = function_callback;
  return true;
}

bool UWBCarpo::Open()
{
  head_can_ptr_->Operate("head_enable_on", std::vector<uint8_t>{});
  head_can_ptr_->LINK_VAR(head_can_ptr_->GetData()->head_enable_on_ack);

  time_t now = time(nullptr);
  while (head_turn_on_ == false && difftime(time(nullptr), now) < 4.0f) {
    std::this_thread::sleep_for(std::chrono::microseconds(30000));
  }

  if (head_turn_on_) {
    INFO("head turn on success.");
  }

  head_can_ptr_->Operate("head_tof_enable_on", std::vector<uint8_t>{});
  head_can_ptr_->LINK_VAR(head_can_ptr_->GetData()->head_tof_enable_on_ack);

  now = time(nullptr);
  while (head_tof_turn_on_ == false && difftime(time(nullptr), now) < 4.0f) {
    std::this_thread::sleep_for(std::chrono::microseconds(30000));
  }

  if (head_tof_turn_on_) {
    INFO("head tof turn on success.");
  }

  rear_can_ptr_->Operate("rear_enable_on", std::vector<uint8_t>{});
  rear_can_ptr_->LINK_VAR(rear_can_ptr_->GetData()->rear_enable_on_ack);

  now = time(nullptr);
  while (rear_turn_on_ == false && difftime(time(nullptr), now) < 4.0f) {
    std::this_thread::sleep_for(std::chrono::microseconds(30000));
  }

  if (rear_turn_on_) {
    INFO("rear turn on success.");
  }

  rear_can_ptr_->Operate("rear_tof_enable_on", std::vector<uint8_t>{});
  rear_can_ptr_->LINK_VAR(rear_can_ptr_->GetData()->rear_tof_enable_on_ack);

  now = time(nullptr);
  while (rear_tof_turn_on_ == false && difftime(time(nullptr), now) < 4.0f) {
    std::this_thread::sleep_for(std::chrono::microseconds(30000));
  }

  if (rear_tof_turn_on_) {
    INFO("rear tof turn on success.");
  }

  if (!rear_tof_turn_on_ || !rear_turn_on_ || !head_tof_turn_on_ || !head_turn_on_) {
    INFO("UWB opened failed.");
    return false;
  }

  head_can_ptr_->LINK_VAR(head_can_ptr_->GetData()->head_data_array);
  head_can_ptr_->LINK_VAR(head_can_ptr_->GetData()->head_tof_data_array);
  rear_can_ptr_->LINK_VAR(rear_can_ptr_->GetData()->rear_data_array);
  rear_can_ptr_->LINK_VAR(rear_can_ptr_->GetData()->rear_tof_data_array);

  INFO("UWB opened successfully");
  return true;
}

bool UWBCarpo::Close()
{
  // head UWB
  head_can_ptr_->Operate("head_enable_off", std::vector<uint8_t>{});
  head_can_ptr_->LINK_VAR(head_can_ptr_->GetData()->head_enable_off_ack);

  time_t now = time(nullptr);
  while (head_turn_on_ == false && difftime(time(nullptr), now) < 4.0f) {
    std::this_thread::sleep_for(std::chrono::microseconds(30000));
  }

  if (head_turn_on_ == false) {
    INFO("Close head uwb success.");
  }

  // head TOF UWB
  head_can_ptr_->Operate("head_tof_enable_off", std::vector<uint8_t>{});
  head_can_ptr_->LINK_VAR(head_can_ptr_->GetData()->head_tof_enable_off_ack);

  now = time(nullptr);
  while (head_tof_turn_on_ == false && difftime(time(nullptr), now) < 4.0f) {
    std::this_thread::sleep_for(std::chrono::microseconds(30000));
  }

  if (head_tof_turn_on_ == false) {
    INFO("Close head tof uwb success.");
  }

  // rear UWB
  rear_can_ptr_->Operate("head_tof_enable_off", std::vector<uint8_t>{});
  rear_can_ptr_->LINK_VAR(rear_can_ptr_->GetData()->rear_enable_off_ack);

  now = time(nullptr);
  while (rear_turn_on_ == false && difftime(time(nullptr), now) < 4.0f) {
    std::this_thread::sleep_for(std::chrono::microseconds(30000));
  }

  if (rear_turn_on_ == false) {
    INFO("Close rear uwb success.");
  }

  // rear TOF UWB
  rear_can_ptr_->Operate("rear_tof_enable_off", std::vector<uint8_t>{});
  rear_can_ptr_->LINK_VAR(rear_can_ptr_->GetData()->rear_tof_enable_off_ack);

  now = time(nullptr);
  while (rear_tof_turn_on_ == false && difftime(time(nullptr), now) < 4.0f) {
    std::this_thread::sleep_for(std::chrono::microseconds(30000));
  }

  if (rear_tof_turn_on_ == false) {
    INFO("Close rear tof uwb success.");
  }

  if (head_turn_on_ || head_tof_turn_on_ ||
    rear_turn_on_ || rear_tof_turn_on_)
  {
    INFO("Close all uwb error.");
    return false;
  }

  INFO("UWB closed successfully");
  return true;
}

bool UWBCarpo::Initialize()
{
  // head UWB
  head_can_ptr_->Operate("head_enable_initial", std::vector<uint8_t>{});
  head_can_ptr_->LINK_VAR(head_can_ptr_->GetData()->head_enable_initial_ack);

  time_t now = time(nullptr);
  while (head_enable_initial_ == false && difftime(time(nullptr), now) < 4.0f) {
    std::this_thread::sleep_for(std::chrono::microseconds(30000));
  }
  if (head_enable_initial_) {
    INFO("head enable initial success.");
  }

  // head TOF UWB
  head_can_ptr_->Operate("head_tof_enable_initial", std::vector<uint8_t>{});
  head_can_ptr_->LINK_VAR(head_can_ptr_->GetData()->head_tof_enable_initial_ack);

  now = time(nullptr);
  while (head_tof_enable_initial_ == false && difftime(time(nullptr), now) < 4.0f) {
    std::this_thread::sleep_for(std::chrono::microseconds(30000));
  }

  if (head_tof_enable_initial_) {
    INFO("head tof enable initial success.");
  }

  // rear UWB
  rear_can_ptr_->Operate("rear_enable_initial", std::vector<uint8_t>{});
  rear_can_ptr_->LINK_VAR(rear_can_ptr_->GetData()->rear_enable_initial_ack);

  now = time(nullptr);
  while (rear_enable_initial_ == false && difftime(time(nullptr), now) < 4.0f) {
    std::this_thread::sleep_for(std::chrono::microseconds(30000));
  }

  if (rear_enable_initial_) {
    INFO("rear enable initial success.");
  }

  // rear TOF UWB
  rear_can_ptr_->Operate("rear_tof_enable_initial", std::vector<uint8_t>{});
  rear_can_ptr_->LINK_VAR(rear_can_ptr_->GetData()->rear_tof_enable_initial_ack);

  now = time(nullptr);
  while (rear_tof_enable_initial_ == false && difftime(time(nullptr), now) < 4.0f) {
    std::this_thread::sleep_for(std::chrono::microseconds(30000));
  }
  if (rear_tof_enable_initial_) {
    INFO("rear tof enable initial success.");
  }

  if (!rear_tof_enable_initial_ || !rear_enable_initial_ ||
    !head_tof_enable_initial_ || !head_enable_initial_)
  {
    INFO("UWB initialized failed.");
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
  INFO_STREAM("~~~~ can0 uwb callback ~~~~~ ");
  INFO_STREAM("    name ==   " << name);

  if (name == "rear_enable_initial_ack") {
    rear_enable_initial_ = true;
    rear_can_ptr_->BREAK_VAR(rear_can_ptr_->GetData()->rear_enable_initial_ack);
  } else if (name == "rear_tof_enable_initial_ack") {
    rear_tof_enable_initial_ = true;
    rear_can_ptr_->BREAK_VAR(rear_can_ptr_->GetData()->rear_tof_enable_initial_ack);
  }

  if (rear_enable_initial_ && rear_tof_enable_initial_) {
    if (name == "rear_enable_on_ack") {
      rear_turn_on_ = true;
      rear_can_ptr_->BREAK_VAR(rear_can_ptr_->GetData()->rear_enable_on_ack);
    } else if (name == "rear_tof_enable_on_ack") {
      rear_tof_turn_on_ = true;
      rear_can_ptr_->BREAK_VAR(rear_can_ptr_->GetData()->rear_tof_enable_on_ack);
    }
  }

  if (name == "rear_enable_off_ack") {
    rear_turn_on_ = false;
    rear_can_ptr_->BREAK_VAR(rear_can_ptr_->GetData()->rear_enable_off_ack);
  } else if (name == "rear_tof_enable_off_ack") {
    rear_tof_turn_on_ = false;
    rear_can_ptr_->BREAK_VAR(rear_can_ptr_->GetData()->rear_tof_enable_off_ack);
  }

  if (rear_tof_turn_on_ && head_turn_on_) {
    if (name == "rear_data_array" || name == "rear_tof_data_array") {
      INFO("Receive rear data or rear tof data");

      // header
      struct timespec ts;
      clock_gettime(CLOCK_REALTIME, &ts);
      ros_uwb_status_.header.frame_id = std::string("uwb_sensor");
      ros_uwb_status_.header.stamp.nanosec = ts.tv_nsec;
      ros_uwb_status_.header.stamp.sec = ts.tv_sec;

      // UWB data
      float dist = data->rear_data_array[0] + (data->rear_data_array[1] << 8);
      short angle = data->rear_data_array[2] + (data->rear_data_array[3] << 8);  // NOLINT
      float nLos = data->rear_data_array[4];
      short rssi_1 = data->rear_data_array[6] + (data->rear_data_array[7] << 8);  // NOLINT
      short rssi_2 = data->rear_data_array[8] + (data->rear_data_array[9] << 8);  // NOLINT
      ros_uwb_status_.data[1].second = ts.tv_sec;
      // ros_uwb_status_.data[1].position = LeftPose(dist, static_cast<float>(angle));
      ros_uwb_status_.data[1].dist = dist;
      ros_uwb_status_.data[1].angle = format_9_7(angle);
      ros_uwb_status_.data[1].n_los = nLos;
      ros_uwb_status_.data[1].rssi_1 = format_8_8(rssi_1);
      ros_uwb_status_.data[1].rssi_2 = format_8_8(rssi_2);

      // TOF data
      float dist_tof = data->rear_tof_data_array[0] + (data->rear_tof_data_array[1] << 8); // NOLINT
      short angle_tof = data->rear_tof_data_array[2] + (data->rear_tof_data_array[3] << 8);   // NOLINT
      float nLos_tof = data->rear_tof_data_array[4];
      float rssi_1_tof = data->rear_tof_data_array[6] + (data->rear_tof_data_array[7] << 8);  // NOLINT
      short rssi_2_tof = data->rear_tof_data_array[8] + (data->rear_tof_data_array[9] << 8);  // NOLINT
      ros_uwb_status_.data[2].second = ts.tv_sec;
      // ros_uwb_status_.data[2].position = RightPose(dist_tof, angle_tof);
      ros_uwb_status_.data[2].dist = dist_tof;
      ros_uwb_status_.data[2].angle = format_9_7(angle_tof);
      ros_uwb_status_.data[2].n_los = nLos_tof;
      ros_uwb_status_.data[2].rssi_1 = format_8_8(rssi_1_tof);
      ros_uwb_status_.data[2].rssi_2 = format_8_8(rssi_2_tof);

      INFO("--------------------[UWB]----------------------");
      INFO("%02X, %02X", data->rear_data_array[0], data->rear_data_array[1]);
      INFO("Current dist : %f", dist);
      INFO("Current angle : %f", format_9_7(angle));
      INFO("Current nLos : %f", nLos);
      INFO("Current rssi_1 : %f", format_8_8(rssi_1));
      INFO("Current rssi_2 : %f", format_8_8(rssi_2));

      INFO("--------------------[TOF]----------------------");
      INFO("%02X, %02X", data->rear_tof_data_array[0], data->rear_tof_data_array[1]);
      INFO("Current dist : %f", dist_tof);
      INFO("Current angle : %f", format_9_7(angle_tof));
      INFO("Current nLos : %f", nLos_tof);
      INFO("Current rssi_1 : %f", format_8_8(rssi_1_tof));
      INFO("Current rssi_2 : %f", format_8_8(rssi_2_tof));
    }
  }
}

void UWBCarpo::HandleCan1Messages(
  std::string & name,
  std::shared_ptr<cyberdog::device::UWBHeadData> data)
{
  INFO_STREAM("~~~~ can1 uwb callback ~~~~~ ");
  INFO_STREAM("    name ==   " << name);

  if (name == "head_enable_initial_ack") {
    head_enable_initial_ = true;
    head_can_ptr_->BREAK_VAR(head_can_ptr_->GetData()->head_enable_initial_ack);
  } else if (name == "head_tof_enable_initial_ack") {
    head_tof_enable_initial_ = true;
    head_can_ptr_->BREAK_VAR(head_can_ptr_->GetData()->head_tof_enable_initial_ack);
  }

  if (head_enable_initial_ && head_tof_enable_initial_) {
    if (name == "head_enable_on_ack") {
      head_turn_on_ = true;
      head_can_ptr_->BREAK_VAR(head_can_ptr_->GetData()->head_enable_on_ack);
    } else if (name == "head_tof_enable_on_ack") {
      head_tof_turn_on_ = true;
      head_can_ptr_->BREAK_VAR(head_can_ptr_->GetData()->head_tof_enable_on_ack);
    }
  }

  if (name == "head_enable_off_ack") {
    head_turn_on_ = false;
    head_can_ptr_->BREAK_VAR(head_can_ptr_->GetData()->head_enable_off_ack);
  } else if (name == "head_tof_enable_off_ack") {
    head_tof_turn_on_ = false;
    head_can_ptr_->BREAK_VAR(head_can_ptr_->GetData()->head_tof_enable_off_ack);
  }

  if (head_tof_turn_on_ && head_turn_on_) {
    if (name == "head_data_array" || name == "head_tof_data_array") {
      INFO("Receive head data or head tof data.");

      // header
      struct timespec ts;
      clock_gettime(CLOCK_REALTIME, &ts);
      ros_uwb_status_.header.frame_id = std::string("uwb_sensor");
      ros_uwb_status_.header.stamp.nanosec = ts.tv_nsec;
      ros_uwb_status_.header.stamp.sec = ts.tv_sec;

      // UWB data
      float dist = data->head_data_array[0] + (data->head_data_array[1] << 8);
      short angle = data->head_data_array[2] + (data->head_data_array[3] << 8);   // NOLINT
      float nLos = data->head_data_array[4];
      short rssi_1 = data->head_data_array[6] + (data->head_data_array[7] << 8);  // NOLINT
      short rssi_2 = data->head_data_array[8] + (data->head_data_array[9] << 8);  // NOLINT
      ros_uwb_status_.data[0].second = ts.tv_sec;
      // ros_uwb_status_.data[0].position = FrontPose(dist, angle);
      ros_uwb_status_.data[0].dist = dist;
      ros_uwb_status_.data[0].angle = format_9_7(angle);
      ros_uwb_status_.data[0].n_los = nLos;
      ros_uwb_status_.data[0].rssi_1 = format_8_8(rssi_1);
      ros_uwb_status_.data[0].rssi_2 = format_8_8(rssi_2);

      // TOF data
      float dist_tof = data->head_tof_data_array[0] + (data->head_tof_data_array[1] << 8);
      short angle_tof = data->head_tof_data_array[2] + (data->head_tof_data_array[3] << 8);   // NOLINT
      float nLos_tof = data->head_tof_data_array[4];
      short rssi_1_tof = data->head_tof_data_array[6] + (data->head_tof_data_array[7] << 8);  // NOLINT
      short rssi_2_tof = data->head_tof_data_array[8] + (data->head_tof_data_array[9] << 8);  // NOLINT
      ros_uwb_status_.data[3].second = ts.tv_sec;
      // ros_uwb_status_.data[3].position = BackPose(dist_tof, angle_tof);
      ros_uwb_status_.data[3].dist = dist_tof;
      ros_uwb_status_.data[3].angle = format_9_7(angle_tof);
      ros_uwb_status_.data[3].n_los = nLos_tof;
      ros_uwb_status_.data[3].rssi_1 = format_8_8(rssi_1_tof);
      ros_uwb_status_.data[3].rssi_2 = format_8_8(rssi_2_tof);

      INFO("--------------------[UWB]----------------------");
      INFO("%02X, %02X", data->head_data_array[0], data->head_data_array[1]);
      INFO("Current dist : %f", dist);
      INFO("Current angle : %f", format_9_7(angle));
      INFO("Current nLos : %f", nLos);
      INFO("Current rssi_1 : %f", format_8_8(rssi_1));
      INFO("Current rssi_2 : %f", format_8_8(rssi_2));

      INFO("--------------------[TOF]----------------------");
      INFO("%02X, %02X", data->head_tof_data_array[0], data->head_tof_data_array[1]);
      INFO("Current dist : %f", dist_tof);
      INFO("Current angle : %f", format_9_7(angle_tof));
      INFO("Current nLos : %f", nLos_tof);
      INFO("Current rssi_1 : %f", format_8_8(rssi_1_tof));
      INFO("Current rssi_2 : %f", format_8_8(rssi_2_tof));
    }
  }
}

void UWBCarpo::RunTask()
{
  while (true) {
    // INFO("UWBCarpo::RunTask ... ");
    UwbRawStatusMsg2Ros();

    if (pose_queue_.empty()) {
      continue;
    }

    // get msg from queue
    auto msg = pose_queue_.front();
    pose_queue_.pop_front();

    // publish msgs
    status_function_(msg);
    std::this_thread::sleep_for(std::chrono::milliseconds(200));
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
        ros_uwb_status_.header.frame_id = std::string("uwb_sensor");
        ros_uwb_status_.header.stamp.nanosec = ts.tv_nsec;
        ros_uwb_status_.header.stamp.sec = ts.tv_sec;

        // 1 head
        // 2 head TOF
        // 3 rear
        // 4 rear TOF
        constexpr int NumberUWB = 4;
        for (int i = 0; i < NumberUWB; i++) {
          UwbSignleStatusMsg uwb;
          uwb.second = ts.tv_sec;
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
      {
        INFO("################## [Current uwb type : Type::HeadTOF] ##################");
        INFO("Current dist : %f", ros_uwb_status_.data[3].dist);
        INFO("Current angle : %f", ros_uwb_status_.data[3].angle);
        INFO("Current nLos : %f", ros_uwb_status_.data[3].n_los);
        INFO("Current rssi_1 : %f", ros_uwb_status_.data[3].rssi_1);
        INFO("Current rssi_2 : %f", ros_uwb_status_.data[3].rssi_2);
        INFO("position x = %f", uwb_posestamped.pose.position.x);
        INFO("position y = %f", uwb_posestamped.pose.position.y);
        INFO("position z = %f", uwb_posestamped.pose.position.z);
      }
      break;

    case Type::HeadUWB:
      {
        INFO("################## [Current uwb type : Type::HeadUWB] ##################");
        INFO("Current dist : %f", ros_uwb_status_.data[0].dist);
        INFO("Current angle : %f", ros_uwb_status_.data[0].angle);
        INFO("Current nLos : %f", ros_uwb_status_.data[0].n_los);
        INFO("Current rssi_1 : %f", ros_uwb_status_.data[0].rssi_1);
        INFO("Current rssi_2 : %f", ros_uwb_status_.data[0].rssi_2);
        INFO("position x = %f", uwb_posestamped.pose.position.x);
        INFO("position y = %f", uwb_posestamped.pose.position.y);
        INFO("position z = %f", uwb_posestamped.pose.position.z);
      }
      break;

    case Type::RearTOF:
      {
        INFO("################## [Current uwb type : Type::RearTOF] ##################");
        INFO("Current dist : %f", ros_uwb_status_.data[2].dist);
        INFO("Current angle : %f", ros_uwb_status_.data[2].angle);
        INFO("Current nLos : %f", ros_uwb_status_.data[2].n_los);
        INFO("Current rssi_1 : %f", ros_uwb_status_.data[2].rssi_1);
        INFO("Current rssi_2 : %f", ros_uwb_status_.data[2].rssi_2);
        INFO("position x = %f", uwb_posestamped.pose.position.x);
        INFO("position y = %f", uwb_posestamped.pose.position.y);
        INFO("position z = %f", uwb_posestamped.pose.position.z);
      }
      break;

    case Type::RearUWB:
      {
        INFO("################## [Current uwb type : Type::RearUWB] ##################");
        INFO("Current dist : %f", ros_uwb_status_.data[1].dist);
        INFO("Current angle : %f", ros_uwb_status_.data[1].angle);
        INFO("Current nLos : %f", ros_uwb_status_.data[1].n_los);
        INFO("Current rssi_1 : %f", ros_uwb_status_.data[1].rssi_1);
        INFO("Current rssi_2 : %f", ros_uwb_status_.data[1].rssi_2);
        INFO("position x = %f", uwb_posestamped.pose.position.x);
        INFO("position y = %f", uwb_posestamped.pose.position.y);
        INFO("position z = %f", uwb_posestamped.pose.position.z);
      }
      break;

    default:
      break;
  }
}

void UWBCarpo::UwbRawStatusMsg2Ros()
{
  // ros_uwb_status_.data[0] // head uwb front
  // ros_uwb_status_.data[3] // head tof back

  // ros_uwb_status_.data[1] // rear uwb left
  // ros_uwb_status_.data[2] // rear tof right
  geometry_msgs::msg::PoseStamped uwb_posestamped;
  bool debug_to_string = false;
  Type type = Type::Unknown;
  constexpr double front_back_threshold = 7.0;
  constexpr auto eps = std::numeric_limits<float>::epsilon();
  double delta = std::fabs(ros_uwb_status_.data[0].rssi_1 - ros_uwb_status_.data[3].rssi_1);

  if (delta > front_back_threshold) {
    constexpr double AoA_F_NMAX = -48.0f;
    constexpr double AoA_F_PMAX = 60.0f;

    if (ros_uwb_status_.data[0].rssi_1 > ros_uwb_status_.data[3].rssi_1) {
      // Head UWB
      if (ros_uwb_status_.data[0].angle > AoA_F_NMAX &&
        ros_uwb_status_.data[0].angle < AoA_F_PMAX)  // NOLINT
      {
        auto pose = FrontPose(ros_uwb_status_.data[0].dist, ros_uwb_status_.data[0].angle);
        struct timespec time_stu;
        clock_gettime(CLOCK_REALTIME, &time_stu);
        uwb_posestamped.header.stamp.nanosec = time_stu.tv_nsec;
        uwb_posestamped.header.stamp.sec = time_stu.tv_sec;
        uwb_posestamped.header.frame_id = "base_link";
        uwb_posestamped.pose.position.x = pose[0];
        uwb_posestamped.pose.position.y = pose[1];
        uwb_posestamped.pose.position.z = pose[2];
        type = Type::HeadUWB;
        pose_queue_.push_back(uwb_posestamped);
        debug_to_string = true;
      } else if (ros_uwb_status_.data[0].angle > AoA_F_PMAX ||  // NOLINT
        helper_functions::rel_eq<double>(ros_uwb_status_.data[0].angle, AoA_F_PMAX, eps))
      {
        // Rear TOF
        // INFO("Rear TOF: ros_uwb_status_.data[2].angle = %f", ros_uwb_status_.data[2].angle);
        auto pose = RightPose(ros_uwb_status_.data[2].dist, ros_uwb_status_.data[2].angle);
        struct timespec time_stu;
        clock_gettime(CLOCK_REALTIME, &time_stu);
        uwb_posestamped.header.stamp.nanosec = time_stu.tv_nsec;
        uwb_posestamped.header.stamp.sec = time_stu.tv_sec;
        uwb_posestamped.header.frame_id = "base_link";
        uwb_posestamped.pose.position.x = pose[0];
        uwb_posestamped.pose.position.y = pose[1];
        uwb_posestamped.pose.position.z = pose[2];
        type = Type::RearTOF;
        pose_queue_.push_back(uwb_posestamped);
        debug_to_string = true;
      } else if (ros_uwb_status_.data[0].angle < AoA_F_NMAX ||  // NOLINT
        helper_functions::rel_eq<double>(ros_uwb_status_.data[0].angle, AoA_F_NMAX, eps))
      {
        // Rear UWB
        // INFO("Rear UWB: ros_uwb_status_.data[1].angle = %f", ros_uwb_status_.data[1].angle);
        auto pose = LeftPose(ros_uwb_status_.data[1].dist, ros_uwb_status_.data[1].angle);
        struct timespec time_stu;
        clock_gettime(CLOCK_REALTIME, &time_stu);
        uwb_posestamped.header.stamp.nanosec = time_stu.tv_nsec;
        uwb_posestamped.header.stamp.sec = time_stu.tv_sec;
        uwb_posestamped.header.frame_id = "base_link";
        uwb_posestamped.pose.position.x = pose[0];
        uwb_posestamped.pose.position.y = pose[1];
        uwb_posestamped.pose.position.z = pose[2];
        type = Type::RearUWB;
        pose_queue_.push_back(uwb_posestamped);
        debug_to_string = true;
      }
    } else {
      constexpr double AoA_B_NMAX = -60.0f;
      constexpr double AoA_B_PMAX = 60.0f;

      if (ros_uwb_status_.data[3].angle > AoA_B_NMAX &&
        ros_uwb_status_.data[3].angle < AoA_B_PMAX)
      {
        // Head TOF
        auto pose = BackPose(ros_uwb_status_.data[3].dist, ros_uwb_status_.data[3].angle);
        struct timespec time_stu;
        clock_gettime(CLOCK_REALTIME, &time_stu);
        uwb_posestamped.header.stamp.nanosec = time_stu.tv_nsec;
        uwb_posestamped.header.stamp.sec = time_stu.tv_sec;
        uwb_posestamped.header.frame_id = "base_link";
        uwb_posestamped.pose.position.x = pose[0];
        uwb_posestamped.pose.position.y = pose[1];
        uwb_posestamped.pose.position.z = pose[2];
        type = Type::HeadTOF;
        pose_queue_.push_back(uwb_posestamped);
        debug_to_string = true;
      } else if (ros_uwb_status_.data[3].angle > AoA_B_PMAX ||  // NOLINT
        helper_functions::rel_eq<double>(ros_uwb_status_.data[3].angle, AoA_B_PMAX, eps))    // NOLINT
      {
        // Rear UWB
        auto pose = LeftPose(ros_uwb_status_.data[1].dist, ros_uwb_status_.data[1].angle);
        struct timespec time_stu;
        clock_gettime(CLOCK_REALTIME, &time_stu);
        uwb_posestamped.header.stamp.nanosec = time_stu.tv_nsec;
        uwb_posestamped.header.stamp.sec = time_stu.tv_sec;
        uwb_posestamped.header.frame_id = "base_link";
        uwb_posestamped.pose.position.x = pose[0];
        uwb_posestamped.pose.position.y = pose[1];
        uwb_posestamped.pose.position.z = pose[2];
        type = Type::RearUWB;
        pose_queue_.push_back(uwb_posestamped);
        debug_to_string = true;
      } else if (ros_uwb_status_.data[3].angle < AoA_B_NMAX ||  // NOLINT
        helper_functions::rel_eq<double>(ros_uwb_status_.data[3].angle, AoA_B_NMAX, eps))    // NOLINT
      {
        // Rear TOF
        // INFO("# Rear TOF: ros_uwb_status_.data[2].angle = %f", ros_uwb_status_.data[2].angle);
        auto pose = RightPose(ros_uwb_status_.data[2].dist, ros_uwb_status_.data[2].angle);
        struct timespec time_stu;
        clock_gettime(CLOCK_REALTIME, &time_stu);
        uwb_posestamped.header.stamp.nanosec = time_stu.tv_nsec;
        uwb_posestamped.header.stamp.sec = time_stu.tv_sec;
        uwb_posestamped.header.frame_id = "base_link";
        uwb_posestamped.pose.position.x = pose[0];
        uwb_posestamped.pose.position.y = pose[1];
        uwb_posestamped.pose.position.z = pose[2];
        type = Type::RearTOF;
        pose_queue_.push_back(uwb_posestamped);
        debug_to_string = true;
      }
    }
  } else {
    constexpr double left_right_threshold = 7.0;
    double delta = std::fabs(ros_uwb_status_.data[1].rssi_1 - ros_uwb_status_.data[2].rssi_1);

    if (delta > left_right_threshold) {
      // rear uwb
      if (ros_uwb_status_.data[1].rssi_1 > ros_uwb_status_.data[2].rssi_1) {
        auto pose = LeftPose(ros_uwb_status_.data[1].dist, ros_uwb_status_.data[1].angle);
        struct timespec time_stu;
        clock_gettime(CLOCK_REALTIME, &time_stu);
        uwb_posestamped.header.stamp.nanosec = time_stu.tv_nsec;
        uwb_posestamped.header.stamp.sec = time_stu.tv_sec;
        uwb_posestamped.header.frame_id = "base_link";
        uwb_posestamped.pose.position.x = pose[0];
        uwb_posestamped.pose.position.y = pose[1];
        uwb_posestamped.pose.position.z = pose[2];
        type = Type::RearUWB;
        pose_queue_.push_back(uwb_posestamped);
        debug_to_string = true;
      } else {
        // rear tof uwb
        auto pose = RightPose(ros_uwb_status_.data[2].dist, ros_uwb_status_.data[2].angle);
        struct timespec time_stu;
        clock_gettime(CLOCK_REALTIME, &time_stu);
        uwb_posestamped.header.stamp.nanosec = time_stu.tv_nsec;
        uwb_posestamped.header.stamp.sec = time_stu.tv_sec;
        uwb_posestamped.header.frame_id = "base_link";
        uwb_posestamped.pose.position.x = pose[0];
        uwb_posestamped.pose.position.y = pose[1];
        uwb_posestamped.pose.position.z = pose[2];
        type = Type::RearTOF;
        pose_queue_.push_back(uwb_posestamped);
        debug_to_string = true;
      }
    }
  }

  if (debug_to_string) {
    Debug2String(type, uwb_posestamped);
  }
}

}  //  namespace device
}  //  namespace cyberdog

PLUGINLIB_EXPORT_CLASS(cyberdog::device::UWBCarpo, cyberdog::device::UWBBase)
