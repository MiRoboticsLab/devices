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
#include <map>
#include <atomic>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "protocol/msg/uwb_raw.hpp"
#include "protocol/msg/uwb_array.hpp"
#include "cyberdog_uwb/uwb_base.hpp"
#include "cyberdog_common/cyberdog_log.hpp"
#include "cyberdog_common/cyberdog_toml.hpp"
#include "cyberdog_common/cyberdog_semaphore.hpp"
#include "cyberdog_system/robot_code.hpp"
#include "rclcpp/rclcpp.hpp"
#include "embed_protocol/embed_protocol.hpp"

#include "cyberdog_uwb/AlgoEKF.h"

namespace cyberdog
{
namespace device
{
namespace EP = cyberdog::embed;
namespace SYS = cyberdog::system;

// Converts from degrees to radians.
constexpr double DegToRad(double deg) {return M_PI * deg / 180.;}

// Converts form radians to degrees.
constexpr double RadToDeg(double rad) {return 180. * rad / M_PI;}

class UWBCarpo : public cyberdog::device::UWBBase
{
  using Signal = cyberdog::common::Semaphore;
  using TomlParse = common::CyberdogToml;
  using Clock = std::chrono::system_clock;

public:
  bool Config() override;
  bool Init(
    std::function<void(UwbSignleStatusMsg)>
    function_callback, bool simulation = false) override;
  int32_t SelfCheck() override;
  bool LowPower() override;
  void Play(
    const std::shared_ptr<protocol::srv::GetUWBMacSessionID::Request> info_request,
    std::shared_ptr<protocol::srv::GetUWBMacSessionID::Response> info_response) override;
  void SetConnectedState(bool connected) override;
  bool RegisterTopic(std::function<void(UwbSignleStatusMsg)> publisher) override;

  bool Open();
  bool Close();

public:
  typedef struct UWB_CellCfg
  {
    std::string name;
    std::string com_file;
    uint16_t mac;
    uint16_t index;
  };
  typedef struct UWBConfig
  {
    bool simulate;
    bool use_static_mac;
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
    std::vector<UWB_CellCfg> uwb_list;
  };
  enum class UWB_Code : int32_t
  {
    kDemoError1 = 21
  };
  typedef struct UWB_Msg
  {
    union {
      uint8_t data[16];
      struct
      {
        uint16_t distance;
        int16_t angle;
        uint8_t nLos;
        uint8_t reserved0;
        int16_t rssi_1;
        int16_t rssi_2;
        uint8_t reserved1[6];
      };
    } __attribute__((packed));
    // can0
    uint8_t enable_initial_ack;
    uint8_t enable_on_ack;
    uint8_t enable_off_ack;
    Signal enable_initial_signal;
    Signal enable_on_signal;
    Signal enable_off_signal;
    Signal data_signal;
    std::atomic<bool> data_received;
    std::atomic<bool> waiting_data;
    uint16_t mac;
    uint16_t index;
  } UWB_Msg;
  enum class Type : int32_t
  {
    HeadUWB  = 0,
    RearUWB,
    RearTOF,
    HeadTOF,
    Unknown
  };

private:
  bool simulation_ {false};
  std::shared_ptr<SYS::CyberdogCode<UWB_Code>> code_{nullptr};
  std::map<std::string, std::shared_ptr<EP::Protocol<UWB_Msg>>> uwb_map_;
  std::map<std::string, int> uwb_index_map_;
  std::atomic<bool> is_working_{false};
  std::function<void(UwbSignleStatusMsg)> topic_pub_;
  // uwb config parameters
  UWBConfig uwb_config_;
  UwbRawStatusMsg ros_uwb_status_;
  std::atomic<uint8_t> data_flag_;
  std::thread simulator_thread_;

  AlgoEKF algo_ekf_;
  struct timespec time_now_, time_pre_;
  float square_deviation_threshold_;
  UwbSignleStatusMsg ros_msg_pub_;

private:
  bool LoadUWBTomlConfig();
  bool Initialize();
  bool InitCAN_Com();
  void UWB_MsgCallback(EP::DataLabel & label, std::shared_ptr<UWB_Msg> data);
  bool TryPublish();
  bool IsSingleStarted(const std::string & name);
  bool IsSingleClosed(const std::string & name);
  bool CheckClosed(int times, const std::string & name);
  // Dimulation Data for debug
  void SimulationThread();
  // Generate random number
  int GenerateRandomNumber(int start, int end);

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
  uint16_t obj_flag_;
  std::atomic<uint16_t> obj_check_;
  bool inited_{false};
  LOGGER_MINOR_INSTANCE("UWBCarpo");
};  //  class UWBCarpo
}   //  namespace device
}   //  namespace cyberdog

#endif  // CYBERDOG_UWB__UWB_PLUGIN_HPP_
