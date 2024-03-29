// Copyright (c) 2023 Beijing Xiaomi Mobile Software Co., Ltd. All rights reserved.
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

#include <memory>
#include <string>
#include <vector>

#include "pluginlib/class_list_macros.hpp"
#include "ament_index_cpp/get_package_share_directory.hpp"
#include "cyberdog_system/robot_code.hpp"

namespace cyberdog
{
namespace device
{
BMSCarpo::BMSCarpo()
{
}

bool BMSCarpo::Config()
{
  return true;
}

bool BMSCarpo::Init(std::function<void(BmsStatusMsg)> function_callback, bool simulation)
{
  inited_ = false;
  const SYS::ModuleCode kModuleCode = SYS::ModuleCode::kBms;
  code_ = std::make_shared<SYS::CyberdogCode<BMS_Code>>(kModuleCode);
  RegisterTopic(function_callback);
  simulator_ = simulation;

  if (simulator_) {
    simulator_thread_ = std::thread(std::bind(&cyberdog::device::BMSCarpo::SimulationThread, this));
    simulator_thread_.detach();
  } else {
    InitializeBmsProtocol();
  }
  Open();
  inited_ = true;
  return true;
}

int32_t BMSCarpo::SelfCheck()
{
  if (!inited_) {
    const SYS::ModuleCode kModuleCode = SYS::ModuleCode::kBms;
    code_ = std::make_shared<SYS::CyberdogCode<BMS_Code>>(kModuleCode);
    ERROR("[%s] Can not do this,you need do init() at first!", __func__);
    return code_->GetKeyCode(SYS::KeyCode::kSelfCheckFailed);
  }
  bool check = battery_->GetData()->data_received;
  INFO("Bms SelfCheck %s", (check ? "successed" : "failed"));
  if (check) {
    return code_->GetKeyCode(SYS::KeyCode::kOK);
  } else {
    return code_->GetKeyCode(SYS::KeyCode::kSelfCheckFailed);
  }
}

bool BMSCarpo::RegisterTopic(std::function<void(BmsStatusMsg)> publisher)
{
  topic_pub_ = publisher;
  return true;
}

bool BMSCarpo::Open()
{
  bool status_ok = true;

  if (!simulator_) {
    if (battery_->GetData()->data_received) {
      INFO("[BMSCarpo] opened successfully");
    } else {
      int retry = 0;
      bool single_status_ok = true;
      while (retry++ < 3) {
        battery_->Operate("bms_enable_on", std::vector<uint8_t>{});
        if (battery_->GetData()->enable_on_signal.WaitFor(1000)) {
          if (!battery_->GetData()->data_received) {
            ERROR(
              "[BMSCarpo] opened failed,can not receive enable on ack ,time[%d]", retry);
            single_status_ok = false;
          } else {
            single_status_ok = true;
            INFO("[BMSCarpo] opened successfully");
            break;
          }
        } else {
          if (battery_->GetData()->bms_enable_on_ack == 0) {
            single_status_ok = true;
            INFO("[BMSCarpo] opened successfully");
            break;
          } else {
            single_status_ok = false;
            ERROR(
              "[BMSCarpo] opened failed, get ack 0x%x!",
              battery_->GetData()->bms_enable_on_ack);
          }
        }
      }
      if (!single_status_ok) {status_ok = false;}
    }
  }
  battery_->GetData()->time_start = std::chrono::system_clock::now();
  is_working_ = (status_ok ? true : false);
  return status_ok;
}

bool BMSCarpo::Close()
{
  bool status_ok = true;

  auto IsClosed = [&]() {
      battery_->GetData()->waiting_data = true;
      bool is_closed = battery_->GetData()->battery_status_signal.WaitFor(2000) ? true : false;
      battery_->GetData()->waiting_data = false;
      battery_->GetData()->data_received = is_closed ? false : true;
      return is_closed;
    };

  if (!simulator_) {
    int retry = 0;
    bool single_status_ok = true;
    while (retry++ < 3) {
      battery_->Operate("bms_enable_off", std::vector<uint8_t>{});
      if (battery_->GetData()->enable_off_signal.WaitFor(1000)) {
        if (IsClosed()) {
          INFO("[BMSCarpo] stoped successfully");
          single_status_ok = true;
          break;
        }
        ERROR(
          "[BMSCarpo] stoped failed,can not receive enable off ack,time[%d]", retry);
        single_status_ok = false;
      } else {
        if (battery_->GetData()->bms_enable_off_ack == 0) {
          INFO("[BMSCarpo] stoped successfully");
          single_status_ok = true;
          break;
        } else {
          single_status_ok = false;
          ERROR(
            "[BMSCarpo] stoped failed, get ack 0x%x!",
            battery_->GetData()->bms_enable_off_ack);
        }
      }
    }
    if (!single_status_ok) {status_ok = false;}
  }
  is_working_ = (status_ok ? false : true);
  return status_ok;
}

void BMSCarpo::ServiceCommand(
  const std::shared_ptr<protocol::srv::BmsCmd::Request> request,
  std::shared_ptr<protocol::srv::BmsCmd::Response> response)
{
  if (request->electric_machine_command ==  // NOLINT
    request->BATTERY_COMMAND_ELECTRIC_MACHINE_POWER_UP)  // NOLINT
  { // NOLINT
    SendCommand(Command::kNormalMode);
  } else if (request->electric_machine_command ==  // NOLINT
    request->BATTERY_COMMAND_ELECTRIC_MACHINE_POWER_DOWN)  // NOLINT
  { // NOLINT
    SendCommand(Command::kTurnOffMotor);
  } else if (request->wireless_charging_command ==  // NOLINT
    request->BATTERY_COMMAND_WIRELESS_CHARGING_TRUN_ON)  // NOLINT
  { // NOLINT
    SendCommand(Command::kTurnOnWirelessCharging);
  } else if (request->wireless_charging_command ==  // NOLINT
    request->BATTERY_COMMAND_WIRELESS_CHARGING_TRUN_OFF)  // NOLINT
  { // NOLINT
    SendCommand(Command::kTurnOffWirelessCharging);
  }
  response->success = true;
}

bool BMSCarpo::LowPower()
{
  return true;
}

void BMSCarpo::BatteryMsgCall(EP::DataLabel & label, std::shared_ptr<BatteryMsg> data)
{
  // set topic msg and pub
  auto MsgCallback = [&]() {
      protocol::msg::BmsStatus message;
      // message.header
      struct timespec time_stu;
      clock_gettime(CLOCK_REALTIME, &time_stu);

      message.header.frame_id = std::string("battery_id");
      message.header.stamp.nanosec = time_stu.tv_nsec;
      message.header.stamp.sec = time_stu.tv_sec;
      message.batt_soc = data->batt_soc;
      message.batt_volt = data->batt_volt;
      message.batt_curr = data->batt_curr;
      message.batt_temp = data->batt_temp;
      message.power_adapter_temp = data->power_adapter_temp;
      message.wireless_charging_temp = data->wireless_charging_temp;
      message.batt_loop_number = data->batt_loop_number;
      message.batt_health = data->batt_health;
      message.batt_st = data->batt_st;

      message.charge_over_current = data->charge_over_current;
      message.discharge_over_current = data->discharge_over_current;
      message.cell_over_voltage = data->cell_over_voltage;
      message.cell_under_voltage = data->cell_under_voltage;
      message.cell_volt_abnormal = data->cell_volt_abnormal;
      message.mos_over_temp = data->mos_over_temp;
      message.discharge_short = data->discharge_short;
      message.fuse = data->fuse;
      message.discharge_over_tmp = data->discharge_over_tmp;
      message.discharge_under_tmp = data->discharge_under_tmp;
      message.charge_over_temp = data->charge_over_temp;
      message.charge_under_temp = data->charge_under_temp;
      message.charge_mos_state = data->charge_mos_state;
      message.discharge_mos_state = data->discharge_mos_state;
      message.chg_mos_fault = data->chg_mos_fault;
      message.dsg_mos_fault = data->dsg_mos_fault;

      message.bms_state_one = data->bms_state1;
      message.power_normal = data->power_normal;
      message.power_wired_charging = data->power_wired_charging;
      message.power_finished_charging = data->power_finished_charging;
      message.power_motor_shutdown = data->power_motor_shutdown;
      message.power_soft_shutdown = data->power_soft_shutdown;
      message.power_wp_place = data->power_wp_place;
      message.power_wp_charging = data->power_wp_charging;
      message.power_expower_supply = data->power_expower_supply;

      static protocol::msg::BmsStatus previous_message = message;
      int soc_jump = previous_message.batt_soc - message.batt_soc;
      if (previous_message.batt_soc != message.batt_soc) {
        if ((soc_jump > 5) || (soc_jump < -5)) {
          ERROR(
            "[Bms]:the soc has junped, soc = %d, volt = %dmV, curr = %dmA, tem = %d,"
            "adap_tem =%d, wireles_tem =%d, loop = %d, health = %d, st =%d, st1 = %d",
            message.batt_soc, message.batt_volt, message.batt_curr, message.batt_temp,
            message.power_adapter_temp, message.wireless_charging_temp, message.batt_loop_number,
            message.batt_health, message.batt_st, message.bms_state_one);
          message = previous_message;
        } else {
          previous_message = message;
        }
      }
      INFO_ONCE(
        "[Bms]:first data soc = %d, volt = %dmV, curr = %dmA, tem = %d, adap_tem =%d, "
        "wireles_tem =%d, loop = %d, health = %d, st =%d, st1 = %d", message.batt_soc,
        message.batt_volt, message.batt_curr, message.batt_temp,
        message.power_adapter_temp, message.wireless_charging_temp, message.batt_loop_number,
        message.batt_health, message.batt_st, message.bms_state_one);
      INFO_MILLSECONDS(
        1000, "[Bms]:soc = %d, volt = %dmV, curr = %dmA, tem = %d, adap_tem =%d, "
        "wireles_tem =%d, loop = %d, health = %d, st =%d, st1 = %d", message.batt_soc,
        message.batt_volt, message.batt_curr, message.batt_temp,
        message.power_adapter_temp, message.wireless_charging_temp, message.batt_loop_number,
        message.batt_health, message.batt_st, message.bms_state_one);
      if (topic_pub_ != nullptr) {
        topic_pub_(message);
      } else {
        ERROR("[Bms]:publisher is nullptr!");
      }
    };

  if (label.name == "bms_enable_on_ack") {
    data->enable_on_signal.Give();
  } else if (label.name == "bms_enable_off_ack") {
    data->enable_off_signal.Give();
  } else if (label.name == "battery_status") {
    if (data->waiting_data) {
      data->battery_status_signal.Give();
    }

    if (!data->data_received) {
      data->data_received = true;
    }
    if (!label.is_full) {
      WARN("[Bms]:data is not full");
    } else {
      // interval callback
      const int kInterval = 800;
      auto now = std::chrono::system_clock::now();
      auto duration =
        std::chrono::duration_cast<std::chrono::milliseconds>(
        now - data->time_start);
      if (duration.count() >= kInterval) {
        MsgCallback();
        data->time_start = std::chrono::system_clock::now();
      }
    }
  } else {
    WARN("unknown msg name %s", label.name.c_str());
  }
}

bool BMSCarpo::SendCommand(const Command & command)
{
  bool success = false;
  switch (command) {
    // 0x01(正常模式）
    case Command::kNormalMode:
      INFO("[BmsProcessor]: command type = Command::kNormalMode");
      success = battery_->Operate("cmd_normal_mode", std::vector<uint8_t>{0x00});
      INFO("[BmsProcessor]: ``````");
      break;

    // 0x02(关闭电机）
    case Command::kTurnOffMotor:
      INFO("[BmsProcessor]: %s", "command type = Command::kTurnOffMotor");
      success = battery_->Operate("cmd_turn_off_motor", std::vector<uint8_t>{0x00});
      break;

    // 0x03(打开无线充电）
    case Command::kTurnOnWirelessCharging:
      INFO("[BmsProcessor]: %s", "command type = Command::kTurnOnWirelessCharging");
      success = battery_->Operate("cmd_turn_on_wireless_charging", std::vector<uint8_t>{0x00});
      break;

    // 0x04(关闭无线充电）
    case Command::kTurnOffWirelessCharging:
      INFO("[BmsProcessor]: %s", "command type = Command::kTurnOffWirelessCharging");
      success = battery_->Operate("cmd_turn_off_wireless_charging", std::vector<uint8_t>{0x00});
      break;
    default:
      break;
  }
  return success;
}

void BMSCarpo::InitializeBmsProtocol()
{
  // Config the battery file
  auto local_share_dir = ament_index_cpp::get_package_share_directory("params");
  auto path = local_share_dir + std::string("/toml_config/device/battery_config.toml");

  // Create Protocol for `BMSStatus` data
  battery_ = std::make_shared<EP::Protocol<BatteryMsg>>(path, false);
  battery_->GetData()->data_received = false;
  INFO("[Bms]:in InitializeBmsprototocol");

  battery_->LINK_VAR(battery_->GetData()->battery_status);
  battery_->LINK_VAR(battery_->GetData()->bms_enable_on_ack);
  battery_->LINK_VAR(battery_->GetData()->bms_enable_off_ack);

  battery_->SetDataCallback(
    std::bind(
      &BMSCarpo::BatteryMsgCall,
      this, std::placeholders::_1, std::placeholders::_2));
}

void BMSCarpo::SimulationThread()
{
  protocol::msg::BmsStatus message;

  while (1) {
    if (is_working_) {
      protocol::msg::BmsStatus message;
      // message.header
      struct timespec time_stu;
      clock_gettime(CLOCK_REALTIME, &time_stu);

      message.header.frame_id = std::string("battery_id");
      message.header.stamp.nanosec = time_stu.tv_nsec;
      message.header.stamp.sec = time_stu.tv_sec;
      message.batt_soc = GenerateRandomNumber(0, 100);       // 电量
      message.batt_volt = GenerateRandomNumber(0, 36);       // 0 V- 36V
      message.batt_curr = GenerateRandomNumber(0, 25);       // 0 MA- 25MA
      message.batt_temp = GenerateRandomNumber(0, 100);      // 0 C- 100 C
      message.batt_loop_number = GenerateRandomNumber(0, 1000);  // batt_loop_number
      message.batt_health = GenerateRandomNumber(0, 100);    // batt_health
      message.power_adapter_temp = GenerateRandomNumber(0, 100);          // key_val
      message.wireless_charging_temp = GenerateRandomNumber(0, 100);  // powerboard_status
      message.batt_st = GenerateRandomNumber(0, 256);  // powerboard_status
      topic_pub_(message);
    }
    std::this_thread::sleep_for(std::chrono::seconds(1));
  }
}

int BMSCarpo::GenerateRandomNumber(int start, int end)
{
  std::random_device rd;   // 将用于为随机数引擎获得种子
  std::mt19937 gen(rd());  // 以播种标准 mersenne_twister_engine
  std::uniform_int_distribution<> dis(start, end);  // [start end]
  return dis(gen);
}

}  //  namespace device
}  //  namespace cyberdog

PLUGINLIB_EXPORT_CLASS(cyberdog::device::BMSCarpo, cyberdog::device::BMSBase)
