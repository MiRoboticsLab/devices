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

#if 0

#include <thread>
#include <chrono>

#include "cyberdog_touch/touch_base.hpp"
#include "pluginlib/class_list_macros.hpp"


namespace cyberdog
{
namespace device
{
class TouchCarpo : public TouchBase
{
  using TouchStatusMsg = protocol::msg::TouchStatus;

private:
  std::function<void(TouchStatusMsg)> status_function_;

public:
  bool Config() override
  {
    return true;
  }

  bool Init(std::function<void(TouchStatusMsg)> f) override
  {
    RegisterTopic(f);
    std::thread t([this]() {
        TouchStatusMsg msg;
        msg.timestamp = 0;
        msg.touch_state = 1;
        while (true) {
          this->status_function_(msg);
          std::cout << "touch once~~~" << std::endl;
          std::this_thread::sleep_for(std::chrono::seconds(2));
        }
      });
    t.detach();
    return true;
  }

  bool SelfCheck() override
  {
    return true;
  }

  bool RegisterTopic(std::function<void(TouchStatusMsg)> f) override
  {
    status_function_ = f;
    return true;
  }
};  // class TouchCarpo

}  // namespace device
}  // namespace cyberdog


PLUGINLIB_EXPORT_CLASS(cyberdog::device::TouchCarpo, cyberdog::device::TouchBase)
#endif
