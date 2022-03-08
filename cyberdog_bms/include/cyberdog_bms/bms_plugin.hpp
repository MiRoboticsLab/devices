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

#ifndef BMS_PLUGIN__BMS_PLUGIN_HPP_
#define BMS_PLUGIN__BMS_PLUGIN_HPP_


#include <memory>
#include <string>
#include <thread>
#include <chrono>

#include "cyberdog_bms/bms_base.hpp"
#include "cyberdog_bms/bms_processor.hpp"
#include "cyberdog_common/cyberdog_log.hpp"
#include "embed_protocol/embed_protocol.hpp"
#include "rclcpp/rclcpp.hpp"


namespace cyberdog {
namespace device {

class BMSCarpo : public cyberdog::device::BMSBase
{
public:
    using BmsStatusMsg = protocol::msg::Bms;

    virtual bool Config() override;
    virtual bool Init() override;
    virtual bool SelfCheck() override;
    virtual bool RegisterTopic(std::function<void(BmsStatusMsg)> function_callback) override;
    virtual void Report(
        const std::shared_ptr<protocol::srv::BmsInfo::Request> request,
        std::shared_ptr<protocol::srv::BmsInfo::Response> response) override;
    
private:
    void RunBmsTask();
    std::function<void(BmsStatusMsg)> status_function_;
    std::shared_ptr<BmsProcessor> bms_processor_ {nullptr};

    std::thread bms_thread_;
    bool initialized_finished_ {false};
}; // class BMSCarpo 

}  // namespace devices
}  // namespace cyberdog

#endif  // BMS_PLUGIN__BMS_PLUGIN_HPP_
