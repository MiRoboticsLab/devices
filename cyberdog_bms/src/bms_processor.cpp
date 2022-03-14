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

void BmsProcessor::HandleBMSMessages(std::string& name, 
    std::shared_ptr<CanProtocolBmsType> data)
{
    if (name != "bms_data_array") {
        WARN("[BmsProcessor]: %s", "HandleBMSMessages error: Type mismatch");
        return;
    }

    // bms_protocol_bridge_->LINK_VAR(bms_protocol_bridge_->GetData()->bms_data_array);
    bms_message_ = ToROSBmsMessage(data);
}

bool BmsProcessor::SendCommand(const Command& command)
{
    bool commmand_send_success = false;
    if (command == Command::kBuzze) {
        INFO("[BmsProcessor]: %s", "command type = Command::kBuzze");
        bms_protocol_bridge_->BREAK_VAR(bms_protocol_bridge_->GetData()->buzze);
        commmand_send_success = bms_protocol_bridge_->Operate("cmd_buzze", std::vector<uint8_t>{0x00});
        bms_protocol_bridge_->LINK_VAR(bms_protocol_bridge_->GetData()->buzze);
    } 
    else if (command == Command::kPowerSupply) {
        INFO("[BmsProcessor]: %s", "Send command type : Command::kPowerSupply");
        bms_protocol_bridge_->BREAK_VAR(bms_protocol_bridge_->GetData()->power_supply);
        commmand_send_success = bms_protocol_bridge_->Operate("cmd_power_supply", std::vector<uint8_t>{0x00});
        bms_protocol_bridge_->LINK_VAR(bms_protocol_bridge_->GetData()->power_supply);
    } 
    else if (command == Command::kDisableCharge) {
        INFO("[BmsProcessor]: %s", "Send command type : Command::kDisableCharge");
        bms_protocol_bridge_->BREAK_VAR(bms_protocol_bridge_->GetData()->disable_charge);
        commmand_send_success = bms_protocol_bridge_->Operate("cmd_disable_charge", std::vector<uint8_t>{0x00});
        bms_protocol_bridge_->LINK_VAR(bms_protocol_bridge_->GetData()->disable_charge);
    } 
    return commmand_send_success;
}

void BmsProcessor::InitializeBmsProtocol()
{
    std::string path = ament_index_cpp::get_package_share_directory("params") +
        "/toml_config/devices/battery.toml";

    bms_protocol_bridge_ = std::make_shared<embed::Protocol<CanProtocolBmsType>>(path, false);
    bms_protocol_bridge_->SetDataCallback(std::bind(&BmsProcessor::HandleBMSMessages, 
        this,std::placeholders::_1, std::placeholders::_2));
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

protocol::msg::Bms BmsProcessor::ToROSBmsMessage(std::shared_ptr<CanProtocolBmsType> data)
{
    protocol::msg::Bms message;
    message.batt_volt = data->batt_volt;
    message.batt_curr = data->batt_curr;
    message.batt_soc  = data->batt_soc;
    message.batt_temp = data->batt_temp;
    message.batt_st   = data->batt_st;
    message.key_val   = data->key_val;
    message.disable_charge    = data->disable_charge;
    message.power_supply      = data->power_supply;
    message.buzze             = data->buzze;
    message.status            = data->status;
    message.batt_health       = data->batt_health;
    message.batt_loop_number  = data->batt_loop_number;
    message.powerboard_status = data->powerboard_status;
    return message;
}

} // namespace devices
} // namespace cyberdog
