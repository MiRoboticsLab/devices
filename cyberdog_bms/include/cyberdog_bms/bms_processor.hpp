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


#ifndef CYBERDOG_BMS__BMS_PROCESSOR_HPP_
#define CYBERDOG_BMS__BMS_PROCESSOR_HPP_

#include "protocol/msg/bms.hpp"
#include "protocol/srv/bms_info.hpp"
#include "embed_protocol/embed_protocol.hpp"
#include "cyberdog_common/cyberdog_log.hpp"
#include "rclcpp/rclcpp.hpp"

namespace cyberdog {
namespace device {

struct CanProtocolBmsType 
{
    uint16_t  batt_volt;
    int16_t   batt_curr;
    uint8_t  batt_soc;
    int16_t   batt_temp;
    uint8_t  batt_st;
    uint8_t  key_val;
    uint8_t  disable_charge;
    uint8_t  power_supply;
    uint8_t  buzze;
    uint8_t  status;
    int8_t   batt_health;
    int16_t   batt_loop_number;
    int8_t   powerboard_status;
};

enum class Command
{
    kBuzze,
    kPowerSupply,
    kDisableCharge
};

class BmsProcessor
{
public:
    BmsProcessor();
    
    // Get protocol go though by emv
    void HandleBMSMessages(std::string& name, std::shared_ptr<CanProtocolBmsType> data);

    // command status for other device
    bool SendCommand(const Command& command);

    // Get BMS message
    protocol::msg::Bms bms_message() const { return bms_message_; }

private:
    void InitializeBmsProtocol();
    void DebugString();
    protocol::msg::Bms ToROSBmsMessage(std::shared_ptr<CanProtocolBmsType> data);

    protocol::msg::Bms bms_message_;
    std::shared_ptr<embed::Protocol<CanProtocolBmsType>> bms_protocol_bridge_;
};
 
}  // namespace devices
}  // namespace cyberdog

#endif // CYBERDOG_BMS__BMS_PROCESSOR_HPP_
