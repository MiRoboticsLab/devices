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

typedef enum{
	LED_OFF,
	LED_BLINK,
	LED_NORMAL,
} LED_STATUS_T;

class BmsProcessor
{
public:
    BmsProcessor();
    
    // Get protocol go though by emv
    void HandleBMSMessages(std::string& name, std::shared_ptr<protocol::msg::Bms> data);

    // Get BMS message
    protocol::msg::Bms bms_message() const { return bms_message_; }

private:
    void InitializeBmsProtocol();
    void SetBuzze(const protocol::msg::Bms::SharedPtr msg);
    void SetPowerSupply(const protocol::msg::Bms::SharedPtr msg);
    void SetDisableCharge(const protocol::msg::Bms::SharedPtr msg);
    void SetShutdown();
    void ControlLEDState(LED_STATUS_T status);
    void DebugString();

    protocol::msg::Bms bms_message_;
    std::shared_ptr<embed::Protocol<protocol::msg::Bms>> bms_protocol_bridge_;
};
 
}  // namespace devices
}  // namespace cyberdog

#endif // CYBERDOG_BMS__BMS_PROCESSOR_HPP_
