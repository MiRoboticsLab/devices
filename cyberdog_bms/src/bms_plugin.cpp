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

#include <memory>
#include <string>
#include <vector>

#include "pluginlib/class_list_macros.hpp"
#include "ament_index_cpp/get_package_share_directory.hpp"

namespace cyberdog
{
namespace device
{

BMSCarpo::BMSCarpo()
{
  constexpr static int kQueueSize = 3;
  queue_.resize(kQueueSize);
}

bool BMSCarpo::Config()
{
  return true;
}

bool BMSCarpo::Init(std::function<void(BmsStatusMsg)> function_callback, bool simulation)
{
  RegisterTopic(function_callback);
  INFO("[Bms]:bind pub callback");
  simulation_ = simulation;

  if (simulation_) {
    RunSimulation();
  } else {
    InitializeBmsProtocol();
  }

  Open();

  initialized_finished_ = true;
  if (!initialized_finished_) {
    INFO("[BMSCarpo]: %s", "Function Init() error.");
    return initialized_finished_;
  }

  bms_thread_ = std::thread(std::bind(&BMSCarpo::RunBmsTask, this));
  bms_thread_.detach();
  INFO("[BMSCarpo]: %s", "BMSCarpo initialize success.");
  return initialized_finished_;
}

bool BMSCarpo::SelfCheck()
{
  INFO("Bms SelfCheck %s", (is_get_real_data_ ? "successed" : "failed"));
  return is_get_real_data_ ? true : false;
}

bool BMSCarpo::RegisterTopic(std::function<void(BmsStatusMsg)> function_callback)
{
  status_function_ = function_callback;
  return true;
}

bool BMSCarpo::Open()
{
  battery_status_ptr_->LINK_VAR(battery_status_ptr_->GetData()->bms_enable_on_ack);
  battery_status_ptr_->Operate("bms_enable_on", std::vector<uint8_t>{});

  time_t time_opened_delay = time(nullptr);
  while (is_open_ == false && difftime(time(nullptr), time_opened_delay) < 2.0f) {
    std::this_thread::sleep_for(std::chrono::microseconds(30000));
    INFO_MILLSECONDS(1000, "difftime = %2f ", difftime(time(nullptr), time_opened_delay));
  }
  if (is_open_ == false) {
    ERROR(
      "Bms open failed,can not receive enable on ack "
      ",maybe can0 channel blocked");
  } else {
    INFO(" bms open successfully ");
  }
  return is_open_;
}

bool BMSCarpo::Close()
{
  battery_status_ptr_->BREAK_VAR(battery_status_ptr_->GetData()->battery_status);
  battery_status_ptr_->LINK_VAR(battery_status_ptr_->GetData()->bms_enable_off_ack);
  bool success = battery_status_ptr_->Operate("bms_enable_off", std::vector<uint8_t>{});
  INFO("bms turn off %s", success ? "success" : "false");
  return success;
}

void BMSCarpo::RunBmsTask()
{
  while (true) {
    // if (queue_.empty()) {
    //   // INFO("Queue is empty.");
    //   continue;
    // }

    // // Get bms state from queue
    // auto msg = queue_.front();
    // queue_.pop_front();

    // Publish bms state
    if (is_get_real_data_) {
      status_function_(ros_bms_message_);
    } else {
      INFO_MILLSECONDS(2000, "[Bms]:pub bms status failed, No real data received");
    }
    // Every one second publish
    std::this_thread::sleep_for(std::chrono::milliseconds(200));
  }
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
  }
  response->success = true;
}

bool BMSCarpo::LowPower()
{
  return true;
}

void BMSCarpo::RunTest()
{
  // kTest
  switch (test_command_) {
    case 1:
      INFO("Test case: Command::kNormalMode");
      SendCommand(Command::kNormalMode);
      break;

    case 2:
      INFO("Test case: Command::kTurnOffMotor");
      SendCommand(Command::kTurnOffMotor);
      break;

    case 3:
      INFO("Test case: Command::kLowPowerConsumption");
      SendCommand(Command::kLowPowerConsumption);
      break;

    case 4:
      INFO("Test case: Command::kSoftShutdown");
      SendCommand(Command::kSoftShutdown);
      break;

    case 5:
      INFO("Test case: Command::kFileTransfer");
      SendCommand(Command::kFileTransfer);
      break;

    case 6:
      INFO("Test case: Command::kOTAUpgrade");
      SendCommand(Command::kOTAUpgrade);
      break;

    case 7:
      INFO("Test case: Command::kTest");
      SendCommand(Command::kTest);
      break;

    default:
      break;
  }
}

void BMSCarpo::StopTest()
{
  INFO("[BmsProcessor]: StopTest.");
  std::lock_guard<std::mutex> lock(test_mutex_);
  test_ = false;
}

void BMSCarpo::StartTest()
{
  INFO("[BmsProcessor]: StartTest.");
  std::lock_guard<std::mutex> lock(test_mutex_);
  test_ = true;
}

void BMSCarpo::SetTestCase(int test_case)
{
  INFO("[BmsProcessor]: SetTestCase.");
  std::lock_guard<std::mutex> lock(test_mutex_);
  test_command_ = test_case;
}

void BMSCarpo::HandleBatteryStatusMessages(std::string & name, std::shared_ptr<BatteryStatus> data)
{
  if (name == "bms_enable_on_ack") {
    INFO_ONCE(" got %s callback", name.c_str());
    is_open_ = true;
    battery_status_ptr_->BREAK_VAR(battery_status_ptr_->GetData()->bms_enable_on_ack);
    battery_status_ptr_->LINK_VAR(battery_status_ptr_->GetData()->battery_status);
  }

  // std::lock_guard<std::mutex> lock(mutex_battery_);
  if (name == "battery_status" || name == "normal_status") {
    can_battery_message_ = *data;
    // 通过电池健康值非0来确定是否收到真实数据
    if (data->battery_status[10] != 0) {
      is_get_real_data_ = true;
    } else {
      WARN("[Bms]get bms date error ,bad data received from can0");
      return;
    }

    std::array<uint8_t, 16> battery_status_data;
    for (size_t i = 0; i < battery_status_data.size(); i++) {
      battery_status_data[i] = data->battery_status[i];
    }
    // Set value for battery status
    SetBatteryStatus(battery_status_data);

    // normal_status
    std::array<uint8_t, 6> normal_status_data;
    for (size_t i = 0; i < normal_status_data.size(); i++) {
      normal_status_data[i] = data->normal_status[i];
    }
    SetNormalStatus(normal_status_data);

    // Convert can message struct to ROS format
    ros_bms_message_ = ToRos(can_battery_message_);
    if (ros_bms_message_.batt_volt == 0) {
      WARN("[Bms]:Battery data may be wrong");
    }
  }
  // auto msg = ToRos(can_battery_message_);
  // queue_.emplace_back(msg);
}

bool BMSCarpo::SendCommand(const Command & command)
{
  bool success = false;
  switch (command) {
    // 0x01(正常模式）
    case Command::kNormalMode:
      INFO("[BmsProcessor]: command type = Command::kNormalMode");
      battery_status_ptr_->BREAK_VAR(battery_status_ptr_->GetData()->battery_status);
      battery_status_ptr_->LINK_VAR(battery_status_ptr_->GetData()->cmd_normal_mode);
      success = battery_status_ptr_->Operate("cmd_normal_mode", std::vector<uint8_t>{0x00});
      battery_status_ptr_->BREAK_VAR(battery_status_ptr_->GetData()->cmd_normal_mode);
      battery_status_ptr_->LINK_VAR(battery_status_ptr_->GetData()->battery_status);
      INFO("[BmsProcessor]: ``````");
      break;

    // 0x02(关闭电机）
    case Command::kTurnOffMotor:
      INFO("[BmsProcessor]: %s", "command type = Command::kTurnOffMotor");
      battery_status_ptr_->BREAK_VAR(battery_status_ptr_->GetData()->battery_status);
      battery_status_ptr_->LINK_VAR(battery_status_ptr_->GetData()->cmd_turn_off_motor);
      success = battery_status_ptr_->Operate("cmd_turn_off_motor", std::vector<uint8_t>{0x00});
      battery_status_ptr_->BREAK_VAR(battery_status_ptr_->GetData()->cmd_turn_off_motor);
      battery_status_ptr_->LINK_VAR(battery_status_ptr_->GetData()->battery_status);
      break;

    // 0x03(低功耗）
    case Command::kLowPowerConsumption:
      INFO("[BmsProcessor]: %s", "command type = Command::kLowPowerConsumption");
      battery_status_ptr_->BREAK_VAR(battery_status_ptr_->GetData()->battery_status);
      battery_status_ptr_->LINK_VAR(battery_status_ptr_->GetData()->cmd_low_power_consumption);
      success =
        battery_status_ptr_->Operate("cmd_low_power_consumption", std::vector<uint8_t>{0x00});
      battery_status_ptr_->BREAK_VAR(battery_status_ptr_->GetData()->cmd_low_power_consumption);
      battery_status_ptr_->LINK_VAR(battery_status_ptr_->GetData()->battery_status);
      break;

    // 0x04(软关机）
    case Command::kSoftShutdown:
      INFO("[BmsProcessor]: %s", "command type = Command::kSoftShutdown");
      battery_status_ptr_->BREAK_VAR(battery_status_ptr_->GetData()->battery_status);
      battery_status_ptr_->LINK_VAR(battery_status_ptr_->GetData()->cmd_soft_shutdown);
      success = battery_status_ptr_->Operate("cmd_soft_shutdown", std::vector<uint8_t>{0x00});
      battery_status_ptr_->BREAK_VAR(battery_status_ptr_->GetData()->cmd_turn_off_motor);
      battery_status_ptr_->LINK_VAR(battery_status_ptr_->GetData()->battery_status);
      break;

    // 文件传输
    case Command::kFileTransfer:
      battery_status_ptr_->BREAK_VAR(battery_status_ptr_->GetData()->battery_status);
      INFO("[BmsProcessor]: %s", "command type = Command::kFileTransfer");
      battery_status_ptr_->LINK_VAR(battery_status_ptr_->GetData()->cmd_file_transfer);
      success = battery_status_ptr_->Operate("cmd_file_transfer", std::vector<uint8_t>{0x00});
      battery_status_ptr_->BREAK_VAR(battery_status_ptr_->GetData()->cmd_file_transfer);
      battery_status_ptr_->LINK_VAR(battery_status_ptr_->GetData()->battery_status);
      break;

    // OTA升级
    case Command::kOTAUpgrade:
      INFO("[BmsProcessor]: %s", "command type = Command::kOTAUpgrade");
      battery_status_ptr_->BREAK_VAR(battery_status_ptr_->GetData()->battery_status);
      battery_status_ptr_->LINK_VAR(battery_status_ptr_->GetData()->cmd_ota_upgrade);
      success = battery_status_ptr_->Operate("cmd_ota_upgrade", std::vector<uint8_t>{0x00});
      battery_status_ptr_->BREAK_VAR(battery_status_ptr_->GetData()->cmd_ota_upgrade);
      battery_status_ptr_->LINK_VAR(battery_status_ptr_->GetData()->battery_status);
      break;

    // 测试
    case Command::kTest:
      battery_status_ptr_->LINK_VAR(battery_status_ptr_->GetData()->normal_status);
      battery_status_ptr_->BREAK_VAR(battery_status_ptr_->GetData()->battery_status);
      INFO("[BmsProcessor]: %s", "command type = Command::kTest");
      battery_status_ptr_->LINK_VAR(battery_status_ptr_->GetData()->cmd_test);
      success = battery_status_ptr_->Operate("cmd_test", std::vector<uint8_t>{0x00});
      battery_status_ptr_->BREAK_VAR(battery_status_ptr_->GetData()->cmd_test);
      battery_status_ptr_->LINK_VAR(battery_status_ptr_->GetData()->battery_status);
      break;

    default:
      break;
  }
  return success;
}

void BMSCarpo::SetBatteryStatus(const std::array<uint8_t, 16> & data)
{
  can_battery_message_.battery_status = data;
}

void BMSCarpo::SetNormalStatus(std::array<uint8_t, 6> data)
{
  can_battery_message_.normal_status = data;
}

protocol::msg::BmsStatus BMSCarpo::ToRos(const BatteryStatus & can_data)
{
  protocol::msg::BmsStatus message;
  // message.header
  struct timespec time_stu;
  clock_gettime(CLOCK_REALTIME, &time_stu);

  message.header.frame_id = std::string("battery_id");
  message.header.stamp.nanosec = time_stu.tv_nsec;
  message.header.stamp.sec = time_stu.tv_sec;

  // v2: 协议
  // 协议改动：
  // 电量%  data[0]
  // 电压mV data[1-2]
  // 电流mA data[3-4]
  // 电池温度（度) data[5]
  // 适配器温度（度）data[6]
  // 无线充温度（度) data[7]
  // 循环次数n data[8-9]
  // 健康度n data[10]
  // 电池状态 data[11-12]
  // 状态 data[13]
  //  - bit 0  正常模式
  //  - bit 1 有线正在充电
  //  - bit 2 充电完成
  //  - bit 3 电机掉电
  //  - bit 4  软关机
  //  - bit 5  无线充在位
  //  - bit 6 无线充电中
  //  - bit7 外部供电
  // 预留 test_data data[14-15]

  message.batt_soc = can_data.battery_status[0];
  message.batt_volt = (can_data.battery_status[1] | can_data.battery_status[2] << 8);
  message.batt_curr = (can_data.battery_status[3] | can_data.battery_status[4] << 8);
  message.batt_temp = can_data.battery_status[5];
  message.batt_loop_number = (can_data.battery_status[8] | can_data.battery_status[9] << 8);
  message.batt_health = can_data.battery_status[10];
  message.batt_st = can_data.battery_status[13];
  message.power_normal = can_data.battery_status[13] & 0x01;
  message.power_wired_charging = can_data.battery_status[13] >> 1 & 0x01;
  message.power_finished_charging = can_data.battery_status[13] >> 2 & 0x01;
  message.power_motor_shutdown = can_data.battery_status[13] >> 3 & 0x01;
  message.power_soft_shutdown = can_data.battery_status[13] >> 4 & 0x01;
  message.power_wp_place = can_data.battery_status[13] >> 5 & 0x01;
  message.power_wp_charging = can_data.battery_status[13] >> 6 & 0x01;
  message.power_expower_supply = can_data.battery_status[13] >> 7 & 0x01;

  static protocol::msg::BmsStatus previous_message = message;
  int soc_jump = previous_message.batt_soc - message.batt_soc;
  if (previous_message.batt_soc != message.batt_soc) {
    if ((soc_jump > 5) || (soc_jump < -5)) {
      ERROR(
        "[Bms]: the battery soc jump form %d to %d, soc = %d, volt = %dmV, curr = %dmA, "
        "temp = %d, st =%d", previous_message.batt_soc, message.batt_soc, message.batt_soc,
        message.batt_volt, message.batt_curr, message.batt_temp, message.batt_st);
      return previous_message;
    } else {
      INFO(
        "[Bms]:The battery soc = %d, volt = %dmV, curr = %dmA, "
        "temp = %d, st =%d", message.batt_soc, message.batt_volt, message.batt_curr,
        message.batt_temp, message.batt_st);
      previous_message = message;
    }
  }
  return message;
}

void BMSCarpo::InitializeBmsProtocol()
{
  // Config the battery file
  auto local_share_dir = ament_index_cpp::get_package_share_directory("params");
  auto path = local_share_dir + std::string("/toml_config/device/battery.toml");

  // Create Protocol for `BMSStatus` data
  battery_status_ptr_ = std::make_shared<cyberdog::embed::Protocol<BatteryStatus>>(path, false);
  INFO("[Bms]:in InitializeBmsprototocol");

  // battery_status_ptr_->LINK_VAR(battery_status_ptr_->GetData()->battery_status);
  // battery_status_ptr_->LINK_VAR(battery_status_ptr_->GetData()->normal_status);

  battery_status_ptr_->SetDataCallback(
    std::bind(
      &BMSCarpo::HandleBatteryStatusMessages,
      this, std::placeholders::_1, std::placeholders::_2));
}

void BMSCarpo::DebugString(const PrintMessageType & type)
{
  switch (type) {
    case PrintMessageType::kBatteryStatus:
      INFO("------------------------ battery_status ------------------------");
      INFO("[BmsProcessor]: 电量: %d", can_battery_message_.battery_status[0]);
      INFO("[BmsProcessor]: 电压: %d", can_battery_message_.battery_status[1]);
      INFO("[BmsProcessor]: 电流: %d", can_battery_message_.battery_status[2]);
      INFO("[BmsProcessor]: 温度: %d", can_battery_message_.battery_status[3]);
      INFO(
        "[BmsProcessor]: 循环次数 : %d",
        (can_battery_message_.battery_status[4] | can_battery_message_.battery_status[5] << 8));
      INFO("[BmsProcessor]: 健康度: %d", can_battery_message_.battery_status[6]);
      INFO("[BmsProcessor]: 状态: %d", can_battery_message_.battery_status[7]);
      break;

    case PrintMessageType::kBatteryTestNormalStatus:
      INFO("------------------------ normal_status ------------------------");
      INFO("[BmsProcessor]: 测试通过: %d", can_battery_message_.normal_status[0]);
      INFO("[BmsProcessor]: 电量正常: %d", can_battery_message_.normal_status[1]);
      INFO("[BmsProcessor]: SC8815正常: %d", can_battery_message_.normal_status[2]);
      INFO("[BmsProcessor]: CYPD3171正常: %d", can_battery_message_.normal_status[3]);
      INFO("[BmsProcessor]: CAN正常: %d", can_battery_message_.normal_status[4]);
      INFO("[BmsProcessor]: 串口正常: %d", can_battery_message_.normal_status[5]);
      break;

    default:
      break;
  }
}

void BMSCarpo::RunSimulation()
{
  std::thread simulation_task([this]()
    {
      while (simulation_) {
        ros_bms_message_.batt_volt = GenerateRandomNumber(0, 36);       // 0 V- 36V
        ros_bms_message_.batt_curr = GenerateRandomNumber(0, 25);       // 0 MA- 25MA
        ros_bms_message_.batt_temp = GenerateRandomNumber(0, 100);      // 0 C- 100 C
        ros_bms_message_.batt_soc = GenerateRandomNumber(0, 100);       // 电量
        ros_bms_message_.key_val = GenerateRandomNumber(0, 5);          // key_val
        ros_bms_message_.batt_health = GenerateRandomNumber(0, 100);    // batt_health
        ros_bms_message_.batt_loop_number = GenerateRandomNumber(0, 1000);  // batt_loop_number
        ros_bms_message_.powerboard_status = GenerateRandomNumber(0, 6);    // powerboard_status
        std::this_thread::sleep_for(std::chrono::seconds(1));
      }
    });

  simulation_task.detach();
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
