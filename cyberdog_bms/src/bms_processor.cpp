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


#include "cyberdog_bms/bms_processor.hpp"
#include "ament_index_cpp/get_package_share_directory.hpp"

#include <string>

namespace cyberdog {
namespace device { 

BmsProcessor::BmsProcessor()
{
    InitializeBmsProtocol();
}

void BmsProcessor::HandleBMSMessages(std::string& name, std::shared_ptr<protocol::msg::Bms> data)
{
    
}

void BmsProcessor::InitializeBmsProtocol()
{
    std::string path = ament_index_cpp::get_package_share_directory("params") +
        "/toml_config/devices/bms.toml";

    bms_protocol_bridge_ = std::make_shared<embed::Protocol<protocol::msg::Bms>>(path, false);
    bms_protocol_bridge_->SetDataCallback(std::bind(&BmsProcessor::HandleBMSMessages, 
        this,std::placeholders::_1, std::placeholders::_2));
}

void BmsProcessor::SetBuzze(const protocol::msg::Bms::SharedPtr msg)
{
    
}

void BmsProcessor::SetPowerSupply(const protocol::msg::Bms::SharedPtr msg)
{
    
}

void BmsProcessor::SetDisableCharge(const protocol::msg::Bms::SharedPtr msg)
{
    
}

void BmsProcessor::SetShutdown()
{
    
}

void BmsProcessor::ControlLEDState(LED_STATUS_T status)
{
    
}

void BmsProcessor::DebugString()
{
    INFO("[BmsProcessor]: BMS volt    : %d", bms_message_.batt_volt);
    INFO("[BmsProcessor]: BMS curr    : %d", bms_message_.batt_curr);
    INFO("[BmsProcessor]: BMS temp    : %d", bms_message_.batt_temp);
    INFO("[BmsProcessor]: BMS soc     : %d", bms_message_.batt_soc);
    INFO("[BmsProcessor]: BMS status  : %d", bms_message_.status);
    INFO("[BmsProcessor]: BMS key_val : %d", bms_message_.key_val);
    INFO("[BmsProcessor]: BMS batt_health       : %d", bms_message_.batt_health);
    INFO("[BmsProcessor]: BMS batt_loop_number  : %d", bms_message_.batt_loop_number);
    INFO("[BmsProcessor]: BMS powerboard_status : %d", bms_message_.powerboard_status);
}

} // namespace devices
} // namespace cyberdog
