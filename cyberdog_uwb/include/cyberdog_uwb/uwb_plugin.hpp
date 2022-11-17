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

#ifndef CYBERDOG_UWB__UWB_PLUGIN_HPP_
#define CYBERDOG_UWB__UWB_PLUGIN_HPP_

#include <shared_mutex>
#include <vector>
#include <array>
#include <memory>
#include <string>
#include <thread>
#include <chrono>
#include <random>
#include <mutex>
#include <queue>
#include <deque>
#include <atomic>
#include <condition_variable>
#include <utility>
#include <tuple>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "protocol/msg/uwb_raw.hpp"
#include "protocol/msg/uwb_array.hpp"
#include "cyberdog_uwb/uwb_base.hpp"
#include "cyberdog_common/cyberdog_log.hpp"
#include "cyberdog_common/cyberdog_toml.hpp"
#include "rclcpp/rclcpp.hpp"
#include "embed_protocol/embed_protocol.hpp"
namespace cyberdog
{
namespace device
{

struct UWBHeadData
{
  uint8_t head_data_array[16];
  uint8_t head_tof_data_array[16];
  uint8_t version[2];

  // can0
  uint8_t head_enable_initial_ack;
  uint8_t head_enable_on_ack;
  uint8_t head_enable_off_ack;
  uint8_t head_tof_enable_initial_ack;
  uint8_t head_tof_enable_on_ack;
  uint8_t head_tof_enable_off_ack;
};

struct UWBRearData
{
  uint8_t rear_data_array[16];
  uint8_t rear_tof_data_array[16];
  uint8_t version[2];

  // can1
  uint8_t rear_enable_initial_ack;
  uint8_t rear_enable_on_ack;
  uint8_t rear_enable_off_ack;
  uint8_t rear_tof_enable_initial_ack;
  uint8_t rear_tof_enable_on_ack;
  uint8_t rear_tof_enable_off_ack;
};

struct UWBConfig
{
  // front_back_threshold = 7
  // left_right_threshold = 7

  // [HeadUWB]
  // AoA_F_NMAX = -48
  // AoA_F_PMAX = 60.0
  // AoA_B_NMAX = -60
  // AoA_B_PMAX = 60.0

  // [RearUWB]
  // AoA_L_NMAX = -40
  // AoA_L_PMAX = 60
  // AoA_R_NMAX = -48
  // AoA_R_PMAX = 48

  double front_back_threshold;
  double left_right_threshold;
  double AoA_F_NMAX;
  double AoA_F_PMAX;
  double AoA_B_NMAX;
  double AoA_B_PMAX;

  double AoA_L_NMAX;
  double AoA_L_PMAX;
  double AoA_R_NMAX;
  double AoA_R_PMAX;

  uint32_t session_id;
  uint16_t controller_mac;
  uint16_t head_tof_mac;
  uint16_t head_uwb_mac;
  uint16_t rear_tof_mac;
  uint16_t rear_uwb_mac;
};

struct UWBConnectionInfo
{
  uint32_t session_id;
  uint16_t controller_mac;
  uint16_t head_tof_mac;
  uint16_t head_uwb_mac;
  uint16_t rear_tof_mac;
  uint16_t rear_uwb_mac;
};

struct WaitingForResponse
{
  explicit WaitingForResponse(int no)
  : task_no_(no)
  {}
  std::mutex mt_;
  std::condition_variable cv_;
  const int task_no_;
};


// Converts from degrees to radians.
constexpr double DegToRad(double deg) {return M_PI * deg / 180.;}

// Converts form radians to degrees.
constexpr double RadToDeg(double rad) {return 180. * rad / M_PI;}


class UWBCarpo : public cyberdog::device::UWBBase
{
public:
  UWBCarpo();
  ~UWBCarpo();
  bool Config() override;
  bool Init(
    std::function<void(UwbSignleStatusMsg)>
    function_callback, bool simulation = false) override;
  bool SelfCheck() override;
  bool LowPower() override;
  void Play(
    const std::shared_ptr<protocol::srv::GetUWBMacSessionID::Request> info_request,
    std::shared_ptr<protocol::srv::GetUWBMacSessionID::Response> info_response) override;
  void SetConnectedState(bool connected) override;
  bool RegisterTopic(
    std::function<void(UwbSignleStatusMsg)> function_callback) override;

  bool Open();
  bool Close();
  bool Initialize();
  bool GetVersion();

private:
  void HandleCan0Messages(std::string & name, std::shared_ptr<cyberdog::device::UWBRearData> data);
  void HandleCan1Messages(std::string & name, std::shared_ptr<cyberdog::device::UWBHeadData> data);


  void RunTask();

  // CAN
  bool InitializeCanCommunication();

  // Dimulation Data for debug
  void RunSimulation();

  // Generate random number
  int GenerateRandomNumber(int start, int end);

  // Convert struct dat to ROS format
  UwbRawStatusMsg ToROS();

  // uwb raw data conver readable data
  int ConvertAngle(const int & angle);
  int ConvertRange(const int & rangle);

  // AOA
  inline float format_9_7(short data)  // NOLINT
  {
    return data * 1.0 / 128;
  }

  // RSSI
  inline float format_8_8(short data)  // NOLINT
  {
    return data * 1.0 / 256;
  }

  std::vector<float> FrontPose(const float & dist, const float & angle);
  std::vector<float> BackPose(const float & dist, const float & angle);
  std::vector<float> RightPose(const float & dist, const float & angle);
  std::vector<float> LeftPose(const float & dist, const float & angle);  // NOLINT

  enum class Type
  {
    HeadUWB  = 0,
    RearUWB,
    RearTOF,
    HeadTOF,
    Unknown
  };

  bool LoadUWBTomlConfig();
  UWBConfig & GetUWBConfig();

  void SetData(const Type & type, const UwbSignleStatusMsg & data);
  void Debug2String(const Type & type, const geometry_msgs::msg::PoseStamped & uwb_posestamped);
  void Debug2String(const UwbSignleStatusMsg & uwb_msg);
  bool UwbRawStatusMsg2Ros();

  std::shared_ptr<cyberdog::embed::Protocol<UWBHeadData>> head_can_ptr_ {nullptr};
  std::shared_ptr<cyberdog::embed::Protocol<UWBRearData>> rear_can_ptr_ {nullptr};
  std::shared_ptr<std::thread> uwb_thread_ {nullptr};

  std::mutex mutex_;
  UwbRawStatusMsg ros_uwb_status_;
  std::shared_mutex raw_data_mutex_0_, raw_data_mutex_1_;
  std::function<void(UwbSignleStatusMsg)> status_function_;

  // uwb config parameters
  UWBConfig uwb_config_;
  toml::value params_toml_;

  // turn on initial flag
  bool head_enable_initial_ {false};
  bool head_tof_enable_initial_ {false};
  bool rear_enable_initial_ {false};
  bool rear_tof_enable_initial_ {false};

  // turn on flag
  bool head_turn_on_ {false};
  bool rear_turn_on_ {false};
  bool head_tof_turn_on_ {false};
  bool rear_tof_turn_on_ {false};

  // version flag
  bool head_version_ {false};
  bool head_tof_version_ {false};
  bool rear_version_ {false};
  bool rear_tof_version_ {false};

  bool simulation_ {false};
  bool initialized_finished_ {false};
  bool enable_initialized_finished_ {false};
  bool use_uwb_ {false};
  bool use_static_mac_ {false};
  UWBConnectionInfo uwb_connect_info_ {0x00000100, 0x0000, 0x0001, 0x0002, 0x0003, 0x0004};

  // geometry_msgs/msg/pose_stamped
  // std::deque<geometry_msgs::msg::PoseStamped> pose_queue_;
  std::deque<UwbSignleStatusMsg> queue_;
  std::atomic_bool activated_ {false};
  bool threading_ {false};
  std::unique_ptr<std::thread> checking_thread_;
  std::queue<int> checking_task_queue_;
  WaitingForResponse wait_open_res_ {1}, wait_close_res_ {2}, wait_init_res_ {0};
  WaitingForResponse task_checking_ {-1};
  bool initializing_ {false};
  void checkResponse();
  bool ifFailThenRetry(
    WaitingForResponse & wfr, int trial_times, int64_t milisec,
    const std::vector<std::tuple<bool *, std::string, bool, std::vector<uint8_t> *>> & checks,
    bool negative = false);
  bool raw_data_updated[2] {false, false};

  LOGGER_MINOR_INSTANCE("UWBCarpo");
};  //  class UWBCarpo
}   //  namespace device
}   //  namespace cyberdog

#endif  // CYBERDOG_UWB__UWB_PLUGIN_HPP_
