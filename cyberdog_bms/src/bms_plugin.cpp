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


#include "cyberdog_bms/bms_plugin.hpp"

namespace cyberdog {
namespace device {

bool BMSCarpo::Config()  
{
    return true;
}

bool BMSCarpo::Init(std::function<void(BmsStatusMsg)> function_callback)
{
    RegisterTopic(function_callback);
    bms_thread_ = std::thread(std::bind(&BMSCarpo::RunBmsTask, this));

    initialized_finished_ = true;
    if (!initialized_finished_) {
        INFO("[BMSCarpo]: %s", "Function Init() error.");
        return initialized_finished_;
    } 

    INFO("[BMSCarpo]: %s", "BMSCarpo initialize success.");
    return initialized_finished_;
}

bool BMSCarpo::SelfCheck() 
{
    return true;
}

bool BMSCarpo::RegisterTopic(std::function<void(BmsStatusMsg)> function_callback)  
{
    status_function_ = function_callback;
    return true;
}

void BMSCarpo::RunBmsTask()
{
    while (true)
    {
        // if () {

        // }
        

        auto message = bms_processor_->bms_message();
        status_function_(message);
        std::this_thread::sleep_for(std::chrono::seconds(2));
    }
}


void BMSCarpo::Report(
        const std::shared_ptr<protocol::srv::BmsInfo::Request> request,
        std::shared_ptr<protocol::srv::BmsInfo::Response> response)
{
    INFO("[cyberdog_bms]: %s", "BMS report message info");
    (void)request;
    response->header.frame_id = "Bms";
    // response->header.stamp    = rclcpp::Time::now();
	response->info.batt_volt  = bms_processor_->bms_message().batt_volt;
	response->info.batt_curr  = bms_processor_->bms_message().batt_curr;
	response->info.batt_temp  = bms_processor_->bms_message().batt_temp;
	response->info.batt_soc   = bms_processor_->bms_message().batt_soc;
	response->info.status     = bms_processor_->bms_message().status;
	response->info.key_val    = bms_processor_->bms_message().key_val;
	response->success         = true;
}

}  // namespace device
}  // namespace cyberdog

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS( cyberdog::device::BMSCarpo, cyberdog::device::BMSBase )
