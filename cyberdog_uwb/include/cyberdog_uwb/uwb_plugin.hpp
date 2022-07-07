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

#include "cyberdog_uwb/uwb_base.hpp"
#include "cyberdog_common/cyberdog_log.hpp"
#include "rclcpp/rclcpp.hpp"
#include "embed_protocol/embed_protocol.hpp"
namespace cyberdog
{
namespace device
{

class UWBCarpo : public cyberdog::device::UWBBase
{
public:
  UWBCarpo();
  virtual bool Config();
  // virtual bool Init(std::function<void(BmsStatusMsg)>
  // function_callback, bool simulation = false);
  virtual bool SelfCheck();
  // virtual bool RegisterTopic(std::function<void(BmsStatusMsg)> function_callback);

private:
  // Dimulation Data for debug
  void RunSimulation();

  // Generate random number
  int GenerateRandomNumber(int start, int end);

};  //  class UWBCarpo
}   //  namespace device
}   //  namespace cyberdog

#endif  // CYBERDOG_UWB__UWB_PLUGIN_HPP_
