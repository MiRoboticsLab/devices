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
  InitializeBmsProtocol();
}

bool BMSCarpo::Config()
{
  return true;
}

bool BMSCarpo::Init(std::function<void(BmsStatusMsg)> function_callback, bool simulation)
{
  RegisterTopic(function_callback);
  bms_thread_ = std::thread(std::bind(&BMSCarpo::RunBmsTask, this));

  simulation_ = simulation;
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
  while (true) {
    // auto message = bms_processor_->bms_message();
    status_function_(bms_message_);
    INFO("BMSCarpo::RunBmsTask ... ");
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

void BMSCarpo::RunTest()
{
    // kNormalMode,
    // kTurnOffMotor,
    // kLowPowerConsumption,
    // kSoftShutdown,

    // // 文件传输
    // // OTA升级
    // kFileTransfer,
    // kOTAUpgrade,

    // // 0x01（测试）
    // kTest
    switch (test_command_)
    {
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

void BMSCarpo::HandleBMSMessages(std::string & name, std::shared_ptr<BMSStatus> data)
{
  can_message_ = *data;
  if (name == "battery_status") {
    INFO("[BmsProcessor]: Receive battery_status message from can.");

     // link BMSStatus data type
     // 电量	电压	电流	温度	循环次数	健康度	故障状态
    can_bridge_->LINK_VAR(can_bridge_->GetData()->battery_status);
    std::array<uint8_t, 8>  battery_status_data;
    for (size_t i = 0; i < battery_status_data.size(); i++) {
      battery_status_data[i] = data->battery_status[i];
    }

    // convert battery_status to ROS
    SetBatteryStatusData(battery_status_data);
  } else if (name == "abnormal_status") {
    INFO("[BmsProcessor]: Receive abnormal_status message from can.");
    can_bridge_->LINK_VAR(can_bridge_->GetData()->abnormal_status);
    std::array<uint8_t, 6>  abnormal_status_data;
    for (size_t i = 0; i < abnormal_status_data.size(); i++) {
      abnormal_status_data[i] = data->abnormal_status[i];
    }

    // convert abnormal_status to ROS
    SetAbnormalStatusData(abnormal_status_data);
  } else if (name == "normal_status") {
    INFO("[BmsProcessor]: Receive normal_status message from can.");
    can_bridge_->LINK_VAR(can_bridge_->GetData()->normal_status);
    std::array<uint8_t, 6>  normal_status_data;
    for (size_t i = 0; i < normal_status_data.size(); i++) {
      normal_status_data[i] = data->normal_status[i];
    }

    // convert normal_status to ROS
    SetNormalStatusData(normal_status_data);
  } else if (name == "normal_mode") {
    // 0x01(正常模式)
    INFO("[BmsProcessor]: Receive normal_mode message from can.");
    can_bridge_->LINK_VAR(can_bridge_->GetData()->normal_mode);

    // convert normal_mode to ROS 
    uint8_t normal_mode = data->normal_mode;
    SetNormalModeData(normal_mode);
  } else if (name == "charging") {
    INFO("[BmsProcessor]: Receive charging message from can.");
    // 0x02(正在充电)
    can_bridge_->LINK_VAR(can_bridge_->GetData()->charging);

    // convert charging to ROS 
    uint8_t charging = data->charging;
    SetChargingData(charging);
  } else if (name == "finished_charging") {
    INFO("[BmsProcessor]: Receive finished_charging message from can.");
    // 0x03(充电完成)
    can_bridge_->LINK_VAR(can_bridge_->GetData()->finished_charging);

    // convert charging to ROS 
    uint8_t finished_charging = data->finished_charging;
    SetChargingData(finished_charging);
  } else if (name == "motor_power_down") {
    INFO("[BmsProcessor]: Receive motor_power_down message from can.");
    // 0x04(电机掉电)
    can_bridge_->LINK_VAR(can_bridge_->GetData()->motor_power_down);
    
    // convert charging to ROS 
    uint8_t motor_power_down = data->motor_power_down;
    SetChargingData(motor_power_down);
  } else if (name == "soft_shutdown") {
    INFO("[BmsProcessor]: Receive soft_shutdown message from can.");
    // 0x05(软关机)
    can_bridge_->LINK_VAR(can_bridge_->GetData()->soft_shutdown);

    // convert charging to ROS 
    uint8_t soft_shutdown = data->soft_shutdown;
    SetChargingData(soft_shutdown);
  }

   // Convert can message struct to ROS format
   bms_message_ = ToRos(can_message_);

   // Print bms status
   DebugString();
}

bool BMSCarpo::SendCommand(const Command & command)
{
  bool success = false;
  switch (command)
  {
    // 0x01(正常模式）
    case Command::kNormalMode:
      INFO("[BmsProcessor]: %s", "command type = Command::kNormalMode");
      can_bridge_->BREAK_VAR(can_bridge_->GetData()->cmd_normal_mode);
      success = can_bridge_->Operate("cmd_normal_mode", std::vector<uint8_t>{0x00});
      can_bridge_->LINK_VAR(can_bridge_->GetData()->cmd_normal_mode);
      break;

    // 0x02(关闭电机）
    case Command::kTurnOffMotor:
      INFO("[BmsProcessor]: %s", "command type = Command::kTurnOffMotor");
      can_bridge_->BREAK_VAR(can_bridge_->GetData()->cmd_turn_off_motor);
      success = can_bridge_->Operate("cmd_turn_off_motor", std::vector<uint8_t>{0x00});
      can_bridge_->LINK_VAR(can_bridge_->GetData()->cmd_turn_off_motor);
      break;

    // 0x03(低功耗）
    case Command::kLowPowerConsumption:
      INFO("[BmsProcessor]: %s", "command type = Command::kLowPowerConsumption");
      can_bridge_->BREAK_VAR(can_bridge_->GetData()->cmd_low_power_consumption);
      success = can_bridge_->Operate("cmd_low_power_consumption", std::vector<uint8_t>{0x00});
      can_bridge_->LINK_VAR(can_bridge_->GetData()->cmd_low_power_consumption);
      break;

    // 0x04(软关机）
    case Command::kSoftShutdown:
      INFO("[BmsProcessor]: %s", "command type = Command::kSoftShutdown");
      can_bridge_->BREAK_VAR(can_bridge_->GetData()->cmd_turn_off_motor);
      success = can_bridge_->Operate("cmd_soft_shutdown", std::vector<uint8_t>{0x00});
      can_bridge_->LINK_VAR(can_bridge_->GetData()->cmd_soft_shutdown);
      break;

    // 文件传输
    case Command::kFileTransfer:
      INFO("[BmsProcessor]: %s", "command type = Command::kFileTransfer");
      can_bridge_->BREAK_VAR(can_bridge_->GetData()->cmd_file_transfer);
      success = can_bridge_->Operate("cmd_file_transfer", std::vector<uint8_t>{0x00});
      can_bridge_->LINK_VAR(can_bridge_->GetData()->cmd_file_transfer);
      break;

    // OTA升级
    case Command::kOTAUpgrade:
      INFO("[BmsProcessor]: %s", "command type = Command::kOTAUpgrade");
      can_bridge_->BREAK_VAR(can_bridge_->GetData()->cmd_ota_upgrade);
      success = can_bridge_->Operate("cmd_ota_upgrade", std::vector<uint8_t>{0x00});
      can_bridge_->LINK_VAR(can_bridge_->GetData()->cmd_ota_upgrade);
      break;

    // 测试
    case Command::kTest:
      INFO("[BmsProcessor]: %s", "command type = Command::kTest");
      can_bridge_->BREAK_VAR(can_bridge_->GetData()->cmd_test);
      success = can_bridge_->Operate("cmd_test", std::vector<uint8_t>{0x00});
      can_bridge_->LINK_VAR(can_bridge_->GetData()->cmd_test);
      break;
  
    default:
      break;
  }
  return success;
}

void BMSCarpo::SetBatteryStatusData(const std::array<uint8_t, 8>& data)
{
  // battery_status[0]   : 电量
  // battery_status[1]   : 电压
  // battery_status[2]   : 电流
  // battery_status[3]   : 温度
  // battery_status[4-5] : 循环次数
  // battery_status[6]   : 健康度
  // battery_status[7]   : 故障状态
  // bms_message_.batt_soc = data[0]; 
  // bms_message_.batt_volt = data[1];           
  // bms_message_.batt_curr = data[2];           
  // bms_message_.batt_temp = data[3];          
  // bms_message_.batt_loop_number = data[4] | data[5] >> 8;
  // bms_message_.batt_health = data[6]; 
  // bms_message_.powerboard_status = data[7];
  can_message_.battery_status = data;
}

void BMSCarpo::SetAbnormalStatusData(const std::array<uint8_t, 6>& data)
{
  can_message_.abnormal_status = data;
}

void BMSCarpo::SetNormalStatusData(const std::array<uint8_t, 6>& data)
{
  can_message_.abnormal_status = data;
}

void BMSCarpo::SetNormalModeData(uint8_t normal_mode)
{
  can_message_.normal_mode = normal_mode;
}

void BMSCarpo::SetChargingData(uint8_t charging)
{
  can_message_.charging = charging;
}

void BMSCarpo::SetFinishedChargingData(uint8_t finished_charging)
{
  can_message_.finished_charging = finished_charging;
}

void BMSCarpo::SetMotorPowerDownData(uint8_t motor_power_down)
{
  can_message_.motor_power_down = motor_power_down;
}

void BMSCarpo::SetSoftShutdownData(uint8_t soft_shutdown)
{
  can_message_.soft_shutdown = soft_shutdown;
}

protocol::msg::Bms BMSCarpo::ToRos(const BMSStatus & can_data)
{
  protocol::msg::Bms message;
  // message.header
  struct timespec time_stu;
  clock_gettime(CLOCK_REALTIME, &time_stu);

  message.header.frame_id = std::string("battery_id");
  message.header.stamp.nanosec = time_stu.tv_nsec;
  message.header.stamp.sec = time_stu.tv_sec;

  // data
  // battery_status[0]   : 电量
  // battery_status[1]   : 电压
  // battery_status[2]   : 电流
  // battery_status[3]   : 温度
  // battery_status[4-5] : 循环次数
  // battery_status[6]   : 健康度
  // battery_status[7]   : 故障状态
  message.batt_soc = can_data.battery_status[0]; 
  message.batt_volt = can_data.battery_status[1];
  message.batt_curr = can_data.battery_status[2];
  message.batt_temp = can_data.battery_status[3];
  message.batt_st = can_data.battery_status[7];
  message.batt_health = can_message_.battery_status[6];
  message.batt_loop_number = (can_message_.battery_status[4] | can_message_.battery_status[5] << 8);
  return message;
}

void BMSCarpo::InitializeBmsProtocol()
{
  // Config the battery file
  auto local_share_dir = ament_index_cpp::get_package_share_directory("params");
  auto path = local_share_dir + std::string("/toml_config/device/battery.toml");

  // Create Protocol for `BMSStatus` data
  can_bridge_ = std::make_shared<cyberdog::embed::Protocol<BMSStatus>>(path, false);

  // link BMSStatus data type
  // 电量	电压	电流	温度	循环次数	健康度	故障状态
  // can_bridge_->LINK_VAR(can_bridge_->GetData()->battery_status);

  // 0x01(正常模式)
  // 0x02(正在充电)
  // 0x03(充电完成)
  // 0x04(电机掉电)
  // 0x05(软关机)
  can_bridge_->LINK_VAR(can_bridge_->GetData()->normal_mode);
  can_bridge_->LINK_VAR(can_bridge_->GetData()->charging);
  can_bridge_->LINK_VAR(can_bridge_->GetData()->finished_charging);
  can_bridge_->LINK_VAR(can_bridge_->GetData()->motor_power_down);
  can_bridge_->LINK_VAR(can_bridge_->GetData()->soft_shutdown);

  // 文件传输
  // OTA升级
  can_bridge_->LINK_VAR(can_bridge_->GetData()->cmd_file_transfer);
  can_bridge_->LINK_VAR(can_bridge_->GetData()->cmd_ota_upgrade);

  // 测试
  can_bridge_->LINK_VAR(can_bridge_->GetData()->cmd_test);

  // 01测试通过	01电量正常	01SC8815正常	01CYPD3171正常	01CAN正常	01串口正常
  // 01测试失败	01电量异常	01SC8815异常	01CYPD3171异常	01CAN异常	01串口异常
  can_bridge_->LINK_VAR(can_bridge_->GetData()->abnormal_status);
  can_bridge_->LINK_VAR(can_bridge_->GetData()->normal_status);

  if (simulation_) {
    RunSimulation();
  } else {
    can_bridge_->SetDataCallback(
    std::bind(
      &BMSCarpo::HandleBMSMessages,
      this, std::placeholders::_1, std::placeholders::_2));
  }
}

void BMSCarpo::DebugString()
{
  INFO("------------------------ battery_status ------------------------");
  // 电量	电压	电流	温度	循环次数	健康度	故障状态
  // battery_status[0]   : 电量
  // battery_status[1]   : 电压
  // battery_status[2]   : 电流
  // battery_status[3]   : 温度
  // battery_status[4-5] : 循环次数
  // battery_status[6]   : 健康度
  // battery_status[7]   : 故障状态
  // std::array<uint8_t, 8> battery_status;
  // INFO("[BmsProcessor]: BMS volt    : %d", bms_message_.batt_volt);
  // INFO("[BmsProcessor]: BMS curr    : %d", bms_message_.batt_curr);
  // INFO("[BmsProcessor]: BMS temp    : %d", bms_message_.batt_temp);
  // INFO("[BmsProcessor]: BMS soc     : %d", bms_message_.batt_soc);
  // INFO("[BmsProcessor]: BMS status  : %d", bms_message_.status);
  // INFO("[BmsProcessor]: BMS key_val : %d", bms_message_.key_val);
  // INFO("[BmsProcessor]: BMS batt_health       : %d", bms_message_.batt_health);
  // INFO("[BmsProcessor]: BMS batt_loop_number  : %d", bms_message_.batt_loop_number);
  // INFO("[BmsProcessor]: BMS powerboard_status : %d", bms_message_.powerboard_status);

  INFO("[BmsProcessor]: 电量	  : %d", can_message_.battery_status[0]);
  INFO("[BmsProcessor]: 电压	  : %d", can_message_.battery_status[1]);
  INFO("[BmsProcessor]: 电流	  : %d", can_message_.battery_status[2]);
  INFO("[BmsProcessor]: 温度	  : %d", can_message_.battery_status[3]);
  INFO("[BmsProcessor]: 循环次数 : %d", (can_message_.battery_status[4] | can_message_.battery_status[5] << 8));
  INFO("[BmsProcessor]: 健康度   : %d", can_message_.battery_status[6]);
  INFO("[BmsProcessor]: 故障状态 : %d", can_message_.battery_status[7]);

  INFO("------------------------ abnormal_status ------------------------");
  // 01测试失败	01电量异常	01SC8815异常	01CYPD3171异常	01CAN异常	01串口异常
  // std::array<uint8_t, 6> abnormal_status;
  INFO("[BmsProcessor]: 测试失败	     : %d", can_message_.abnormal_status[0]);
  INFO("[BmsProcessor]: 电量异常	     : %d", can_message_.abnormal_status[1]);
  INFO("[BmsProcessor]: SC8815异常	  : %d", can_message_.abnormal_status[2]);
  INFO("[BmsProcessor]: CYPD3171异常	: %d", can_message_.abnormal_status[3]);
  INFO("[BmsProcessor]: CAN异常	      : %d", can_message_.abnormal_status[4]);
  INFO("[BmsProcessor]: 串口异常	     : %d", can_message_.abnormal_status[5]);
  

  INFO("------------------------ normal_status ------------------------");
  // 01测试通过	01电量正常	01SC8815正常	01CYPD3171正常	01CAN正常	01串口正常
  // std::array<uint8_t, 6> normal_status;
  INFO("[BmsProcessor]: 测试通过	     : %d", can_message_.normal_status[0]);
  INFO("[BmsProcessor]: 电量正常	     : %d", can_message_.normal_status[1]);
  INFO("[BmsProcessor]: SC8815正常	  : %d", can_message_.normal_status[2]);
  INFO("[BmsProcessor]: CYPD3171正常	: %d", can_message_.normal_status[3]);
  INFO("[BmsProcessor]: CAN正常	      : %d", can_message_.normal_status[4]);
  INFO("[BmsProcessor]: 串口正常	     : %d", can_message_.normal_status[5]);

  INFO("------------------------ power_status ------------------------");
  // 0x01(正常模式)
  // 0x02(正在充电)
  // 0x03(充电完成)
  // 0x04(电机掉电)
  // 0x05(软关机)
  // uint8_t normal_mode;
  // uint8_t charging;
  // uint8_t finished_charging;
  // uint8_t motor_power_down;
  // uint8_t soft_shutdown;
  INFO("[BmsProcessor]: normal_mode	      : %d", can_message_.normal_mode);
  INFO("[BmsProcessor]: charging	        : %d", can_message_.charging);
  INFO("[BmsProcessor]: finished_charging	: %d", can_message_.finished_charging);
  INFO("[BmsProcessor]: motor_power_down	: %d", can_message_.motor_power_down);
  INFO("[BmsProcessor]: soft_shutdown	    : %d", can_message_.soft_shutdown);
}

void BMSCarpo::RunSimulation()
{
  bms_message_.batt_volt = GenerateRandomNumber(0, 36);           // 0 V- 36V
  bms_message_.batt_curr = GenerateRandomNumber(0, 25);           // 0 MA- 25MA
  bms_message_.batt_temp = GenerateRandomNumber(0, 100);          // 0 C- 100 C
  bms_message_.batt_soc = GenerateRandomNumber(0, 100);           // 电量
  bms_message_.key_val = GenerateRandomNumber(0, 5);              // key_val
  bms_message_.batt_health = GenerateRandomNumber(0, 100);        // batt_health
  bms_message_.batt_loop_number = GenerateRandomNumber(0, 1000);  // batt_loop_number
  bms_message_.powerboard_status = GenerateRandomNumber(0, 6);    // powerboard_status
}

int BMSCarpo::GenerateRandomNumber(int start, int end)
{
  std::random_device rd;  // 将用于为随机数引擎获得种子
  std::mt19937 gen(rd()); // 以播种标准 mersenne_twister_engine
  std::uniform_int_distribution<> dis(start, end); // [start end]
  return dis(gen);
}

}  //  namespace device
}  //  namespace cyberdog

PLUGINLIB_EXPORT_CLASS(cyberdog::device::BMSCarpo, cyberdog::device::BMSBase)
