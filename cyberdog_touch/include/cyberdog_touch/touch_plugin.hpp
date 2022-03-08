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

#ifndef TOUCH_PLUGIN__TOUCH_PLUGIN_HPP_
#define TOUCH_PLUGIN__TOUCH_PLUGIN_HPP_


#include <memory>
#include <string>

#include "cyberdog_touch/touch_base.hpp"
#include "rclcpp/rclcpp.hpp"


namespace cyberdog {
namespace device {

class TouchCarpo : public cyberdog::device::TouchBase
{
public:
    using TouchStatusMsg = protocol::msg::TouchStatus;

    virtual bool Config() override;
    virtual bool Init(std::function<void(TouchStatusMsg)> f) override;
    virtual bool SelfCheck() override;
    virtual bool RegisterTopic(std::function<void(TouchStatusMsg)> f) override;
    
private:
    std::function<void(TouchStatusMsg)> status_function_;

}; // class TouchCarpo 

}  // namespace devices
}  // namespace cyberdog

#endif  // TOUCH_PLUGIN__TOUCH_PLUGIN_HPP_
