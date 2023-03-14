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

#ifndef CYBERDOG_TOUCH__TOUCH_PLUGIN_HPP_
#define CYBERDOG_TOUCH__TOUCH_PLUGIN_HPP_


#include <memory>
#include <string>

#include "cyberdog_touch/touch_sensor_handler.hpp"
#include "cyberdog_touch/touch_base.hpp"
#include "cyberdog_common/cyberdog_log.hpp"
#include "cyberdog_system/robot_code.hpp"

#include "rclcpp/rclcpp.hpp"


namespace cyberdog
{
namespace device
{
namespace SYS = cyberdog::system;

class TouchCarpo : public cyberdog::device::TouchBase
{
public:
  using TouchStatusMsg = protocol::msg::TouchStatus;

  virtual bool Config();
  virtual bool Init(std::function<void(TouchStatusMsg)> function_callback, bool simulation = false);
  virtual int32_t SelfCheck();
  virtual bool RegisterTopic(std::function<void(TouchStatusMsg)> function_callback);
  virtual bool LowPower();

public:
  enum class TouchCode : int32_t
  {
    kDemoError1 = 21
  };

private:
  std::shared_ptr<SYS::CyberdogCode<TouchCode>> code_{nullptr};

private:
  void RunTouchTask();

  // Dimulation Data for debug
  void RunSimulation();

  // Generate random number
  int GenerateRandomNumber(int start, int end);

  std::function<void(TouchStatusMsg)> status_function_;
  std::shared_ptr<TouchSensorHandler> touch_handler_;
  std::thread touch_thread_;
  bool simulation_ {false};
};  //  class TouchCarpo

}  //  namespace device
}  //  namespace cyberdog

#endif  // CYBERDOG_TOUCH__TOUCH_PLUGIN_HPP_
