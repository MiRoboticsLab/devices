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

#include <thread>
#include <chrono>
#include <utility>
#include <memory>

#include "cyberdog_touch/touch_plugin.hpp"
#include "pluginlib/class_list_macros.hpp"

namespace cyberdog
{
namespace device
{

bool TouchCarpo::Config()
{
  return true;
}

bool TouchCarpo::Init(std::function<void(TouchStatusMsg)> function_callback)
{
  touch_handler_ = std::make_shared<TouchSensorHandler>();
  touch_thread_ = std::thread(std::bind(&TouchCarpo::RunTouchTask, this));
  initialized_finished_ = RegisterTopic(function_callback);

  if (!initialized_finished_) {
    WARN("[TouchCarpo]: %s", "Function Init() error.");
    return initialized_finished_;
  }

  INFO("[TouchCarpo]: %s", "TouchCarpo initialize success.");
  return initialized_finished_;
}

bool TouchCarpo::SelfCheck()
{
  return true;
}

bool TouchCarpo::RegisterTopic(std::function<void(TouchStatusMsg)> function_callback)
{
  status_function_ = function_callback;
  return true;
}

void TouchCarpo::RunTouchTask()
{
  if (!initialized_finished_) {
    WARN("[TouchCarpo]: %s", "touch sensor handler create failed!");
    return;
  }

  int ret = -1;
  int ret_count, count = 1;
  struct input_event * touch_event = (struct input_event *)
    malloc(sizeof(struct input_event) * count);

  while (true) {
    ret_count = touch_handler_->pollTouchEvents(touch_event, count);
    if (ret_count <= 0) {
      continue;
    }

    if ((touch_event->type == EV_KEY)) {
      ret = touch_event->code - GESTURE_XM_ADDR;
      TouchStatusMsg message;
      message.touch_state = ret;
      struct timespec ts;
      clock_gettime(CLOCK_REALTIME, &ts);
      message.timestamp = ts.tv_sec * 1000000000 + ts.tv_nsec;
      INFO("[TouchCarpo]: touch sensor data received: 0x%x", message.touch_state);
      status_function_(std::move(message));
      std::this_thread::sleep_for(std::chrono::seconds(2));
    }
  }
}

}  //  namespace device
}  //  namespace cyberdog


PLUGINLIB_EXPORT_CLASS(cyberdog::device::TouchCarpo, cyberdog::device::TouchBase)
