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

#include <array>
#include <memory>
#include <string>
#include <thread>
#include <chrono>
#include <random>
#include <mutex>

#include "protocol/msg/uwb_raw.hpp"
#include "protocol/msg/uwb_array.hpp"
#include "cyberdog_uwb/uwb_base.hpp"
#include "cyberdog_common/cyberdog_log.hpp"
#include "rclcpp/rclcpp.hpp"
#include "embed_protocol/embed_protocol.hpp"
namespace cyberdog
{
namespace device
{

struct UWBData
{
  uint8_t head[64];   // Data[0-1]	  distance距离
                      // Data[2-3]	  aoa 角度
                      // Data[4]	    nLos  多径标志
                      // Data[5]	    reserved
                      // Data[6-7]	  rssi_1
                      // Data[8-9]	  rssi_2
                      // Data[10-15]	reserved


  uint8_t head_tof[64];   // Data[0-1]	  distance距离
                      // Data[2-3]	  aoa 角度
                      // Data[4]	    nLos  多径标志
                      // Data[5]	    reserved
                      // Data[6-7]	  rssi_1
                      // Data[8-9]	  rssi_2
                      // Data[10-15]	reserved

  uint8_t rear[64];   // Data[0-1]	  distance距离
                      // Data[2-3]	  aoa 角度
                      // Data[4]	    nLos  多径标志
                      // Data[5]	    reserved
                      // Data[6-7]	  rssi_1
                      // Data[8-9]	  rssi_2
                      // Data[10-15]	reserved

  uint8_t rear_tof[64];   // Data[0-1]	  distance距离
                      // Data[2-3]	  aoa 角度
                      // Data[4]	    nLos  多径标志
                      // Data[5]	    reserved
                      // Data[6-7]	  rssi_1
                      // Data[8-9]	  rssi_2
                      // Data[10-15]	reserved

  uint8_t version[2];

  // can0
  uint8_t head_enable_initial_ack;
  uint8_t head_enable_on_ack;
  uint8_t head_enable_off_ack;
  uint8_t head_tof_enable_initial_ack;
  uint8_t head_tof_enable_on_ack;
  uint8_t head_tof_enable_off_ack;

  // can1
  uint8_t rear_enable_initial_ack;
  uint8_t rear_enable_on_ack;
  uint8_t rear_enable_off_ack;
  uint8_t rear_tof_enable_initial_ack;
  uint8_t rear_tof_enable_on_ack;
  uint8_t rear_tof_enable_off_ack;
};

class UWBCarpo : public cyberdog::device::UWBBase
{
public:
  UWBCarpo();
  virtual bool Config() override;
  virtual bool Init(std::function<void(UwbRawStatusMsg)>
    function_callback, bool simulation = false) override;
  virtual bool SelfCheck() override;
  virtual bool RegisterTopic(std::function<void(UwbRawStatusMsg)> function_callback) override;

  bool Open();
  bool Close();
  bool Initialize();
  bool GetVersion();

private:
  void HandleCan0Messages(std::string & name, std::shared_ptr<cyberdog::device::UWBData> data);
  void HandleCan1Messages(std::string & name, std::shared_ptr<cyberdog::device::UWBData> data);

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
  int ConvertAngle(const int& angle);
  int ConvertRange(const int& rangle);

  std::shared_ptr<cyberdog::embed::Protocol<UWBData>> head_can_ptr_ {nullptr};
  std::shared_ptr<cyberdog::embed::Protocol<UWBData>> rear_can_ptr_ {nullptr};
  std::shared_ptr<std::thread> uwb_thread_ {nullptr}; 

  std::mutex mutex_;
  UwbRawStatusMsg ros_uwb_status_;
  std::function<void(UwbRawStatusMsg)> status_function_;

  // uwb data
  UWBData uwb_head_data_;
  UWBData uwb_rear_data_;
  UWBData uwb_tof_head_data_;
  UWBData uwb_tof_rear_data_;

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
};  //  class UWBCarpo
}   //  namespace device
}   //  namespace cyberdog

#endif  // CYBERDOG_UWB__UWB_PLUGIN_HPP_
