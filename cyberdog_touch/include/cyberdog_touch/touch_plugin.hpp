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

#include "cyberdog_touch/touch_sensor_handler.hpp"
#include "cyberdog_touch/touch_base.hpp"
#include "cyberdog_common/cyberdog_log.hpp"

#include "rclcpp/rclcpp.hpp"


namespace cyberdog {
namespace device {

class TouchCarpo : public cyberdog::device::TouchBase
{
public:
    using TouchStatusMsg = protocol::msg::TouchStatus;

    virtual bool Config() override;
    virtual bool Init(std::function<void(TouchStatusMsg)> function_callback) override;
    virtual bool SelfCheck() override;
    virtual bool RegisterTopic(std::function<void(TouchStatusMsg)> function_callback) override;
    
private:
    void RunTouchTask();

    std::function<void(TouchStatusMsg)> status_function_;
    std::shared_ptr<TouchSensorHandler> touch_handler_;
    std::thread touch_thread_;

     bool initialized_finished_ {false};

}; // class TouchCarpo 

}  // namespace devices
}  // namespace cyberdog

#endif  // TOUCH_PLUGIN__TOUCH_PLUGIN_HPP_
