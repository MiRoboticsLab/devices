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
#include "ament_index_cpp/get_package_share_directory.hpp"

namespace cyberdog {
namespace device {

BMSCarpo::BMSCarpo()
{
    InitializeBmsProtocol();
}

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
        // auto message = bms_processor_->bms_message();
        status_function_(bms_message_);

        std::cout << "BMSCarpo::RunBmsTask()" << std::endl;
        std::this_thread::sleep_for(std::chrono::seconds(2));
    }
}


void BMSCarpo::Report(
        const std::shared_ptr<protocol::srv::BmsInfo::Request> request,
        std::shared_ptr<protocol::srv::BmsInfo::Response> response)
{
    // INFO("[cyberdog_bms]: %s", "BMS report message info");
    // (void)request;
    // response->header.frame_id = "Bms";
    // // response->header.stamp    = rclcpp::Time::now();
	// response->info.batt_volt  = bms_processor_->bms_message().batt_volt;
	// response->info.batt_curr  = bms_processor_->bms_message().batt_curr;
	// response->info.batt_temp  = bms_processor_->bms_message().batt_temp;
	// response->info.batt_soc   = bms_processor_->bms_message().batt_soc;
	// response->info.status     = bms_processor_->bms_message().status;
	// response->info.key_val    = bms_processor_->bms_message().key_val;
	// response->success         = true;
}


void BMSCarpo::HandleBMSMessages(std::string& name, 
    std::shared_ptr<CanProtocolBmsType> data)
{

    bms_message_ = ToRos(*data);
    DebugString();

    // bms_can_bridge_->LINK_VAR(bms_can_bridge_->GetData()->bms_data_array);
    // bms_message_ = ToROSBmsMessage(data);
}

bool BMSCarpo::SendCommand(const Command& command)
{
    bool commmand_send_success = false;
    if (command == Command::kBuzze) {
        INFO("[BmsProcessor]: %s", "command type = Command::kBuzze");
        bms_can_bridge_->BREAK_VAR(bms_can_bridge_->GetData()->buzze);
        commmand_send_success = bms_can_bridge_->Operate("cmd_buzze", std::vector<uint8_t>{0x00});
        bms_can_bridge_->LINK_VAR(bms_can_bridge_->GetData()->buzze);
    } 
    else if (command == Command::kPowerSupply) {
        INFO("[BmsProcessor]: %s", "Send command type : Command::kPowerSupply");
        bms_can_bridge_->BREAK_VAR(bms_can_bridge_->GetData()->power_supply);
        commmand_send_success = bms_can_bridge_->Operate("cmd_power_supply", std::vector<uint8_t>{0x00});
        bms_can_bridge_->LINK_VAR(bms_can_bridge_->GetData()->power_supply);
    } 
    else if (command == Command::kDisableCharge) {
        INFO("[BmsProcessor]: %s", "Send command type : Command::kDisableCharge");
        bms_can_bridge_->BREAK_VAR(bms_can_bridge_->GetData()->disable_charge);
        commmand_send_success = bms_can_bridge_->Operate("cmd_disable_charge", std::vector<uint8_t>{0x00});
        bms_can_bridge_->LINK_VAR(bms_can_bridge_->GetData()->disable_charge);
    } 
    return commmand_send_success;
}

protocol::msg::Bms BMSCarpo::ToRos(const CanProtocolBmsType& can_data)
{
    protocol::msg::Bms message;
    // message.header
    struct timespec time_stu;
    clock_gettime(CLOCK_REALTIME, &time_stu);
    message.header.frame_id = std::string("battery_id");
    message.header.stamp.nanosec = time_stu.tv_nsec;
    message.header.stamp.sec = time_stu.tv_sec;

    // data
    message.batt_volt = can_data.batt_volt;
    message.batt_curr = can_data.batt_curr;
    message.batt_soc  = can_data.batt_soc;
    message.batt_temp = can_data.batt_temp;
    message.batt_st   = can_data.batt_st;
    message.key_val   = can_data.key_val;
    message.disable_charge    = can_data.disable_charge;
    message.power_supply      = can_data.power_supply;
    message.buzze             = can_data.buzze;
    message.status            = can_data.status;
    message.batt_health       = can_data.batt_health;
    message.batt_loop_number  = can_data.batt_loop_number;
    message.powerboard_status = can_data.powerboard_status;
    return message;
}

void BMSCarpo::InitializeBmsProtocol()
{
    // Config the battery file
    auto local_share_dir = ament_index_cpp::get_package_share_directory("params");
    auto path = local_share_dir + std::string("/toml_config/device/battery.toml");

    // Create Protocol for `CanProtocolBmsType` data
    bms_can_bridge_ = std::make_shared<cyberdog::embed::Protocol<CanProtocolBmsType>>(path, false);

    // link CanProtocolBmsType data type
    bms_can_bridge_->LINK_VAR(bms_can_bridge_->GetData()->batt_volt);
    bms_can_bridge_->LINK_VAR(bms_can_bridge_->GetData()->batt_curr);
    bms_can_bridge_->LINK_VAR(bms_can_bridge_->GetData()->batt_soc);
    bms_can_bridge_->LINK_VAR(bms_can_bridge_->GetData()->batt_temp);
    bms_can_bridge_->LINK_VAR(bms_can_bridge_->GetData()->batt_st);
    bms_can_bridge_->LINK_VAR(bms_can_bridge_->GetData()->key_val);
    bms_can_bridge_->LINK_VAR(bms_can_bridge_->GetData()->disable_charge);
    bms_can_bridge_->LINK_VAR(bms_can_bridge_->GetData()->power_supply);
    bms_can_bridge_->LINK_VAR(bms_can_bridge_->GetData()->buzze);
    bms_can_bridge_->LINK_VAR(bms_can_bridge_->GetData()->status);
    bms_can_bridge_->LINK_VAR(bms_can_bridge_->GetData()->batt_health);
    bms_can_bridge_->LINK_VAR(bms_can_bridge_->GetData()->batt_loop_number);
    bms_can_bridge_->LINK_VAR(bms_can_bridge_->GetData()->powerboard_status);

    bms_can_bridge_->SetDataCallback(std::bind(&BMSCarpo::HandleBMSMessages, 
        this,std::placeholders::_1, std::placeholders::_2));
}

void BMSCarpo::DebugString()
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

}  // namespace device
}  // namespace cyberdog

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS( cyberdog::device::BMSCarpo, cyberdog::device::BMSBase )