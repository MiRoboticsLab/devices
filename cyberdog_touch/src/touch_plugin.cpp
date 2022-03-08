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

#include "cyberdog_touch/touch_plugin.hpp"

namespace cyberdog {
namespace device {

bool TouchCarpo::Config()  
{
    return true;
}

bool TouchCarpo::Init(std::function<void(TouchStatusMsg)> f)  
{
    RegisterTopic(f);
    std::thread t([this](){
        TouchStatusMsg msg;
        msg.timestamp = 0;
        msg.touch_state = 1;
        while (true)
        {
        this->status_function_(msg);
        std::cout << "touch once~~~" << std::endl;
        std::this_thread::sleep_for(std::chrono::seconds(2));
        }
    });
    t.detach();
    return true;
}

bool TouchCarpo::SelfCheck()  
{
    return true;
}


bool TouchCarpo::RegisterTopic(std::function<void(TouchStatusMsg)> f)  
{
    status_function_ = f;
    return true;
}

}  // namespace device
}  // namespace cyberdog

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS( cyberdog::device::TouchCarpo, cyberdog::device::TouchBase)