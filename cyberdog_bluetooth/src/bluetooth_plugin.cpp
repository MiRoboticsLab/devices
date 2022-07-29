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


#include "cyberdog_bluetooth/bluetooth_plugin.hpp"

#include <memory>
#include <string>
#include <vector>

#include "pluginlib/class_list_macros.hpp"
#include "ament_index_cpp/get_package_share_directory.hpp"

namespace cyberdog
{
namespace device
{

BluetoothCarpo::BluetoothCarpo()
{
}

bool BluetoothCarpo::Config()
{
  return true;
}

void BluetoothCarpo::RunSimulation()
{
  std::thread simulation_task([this]()
    {
      while (simulation_) {
        // ros_bms_message_.batt_volt = GenerateRandomNumber(0, 36);       // 0 V- 36V
        // ros_bms_message_.batt_curr = GenerateRandomNumber(0, 25);       // 0 MA- 25MA
        // ros_bms_message_.batt_temp = GenerateRandomNumber(0, 100);      // 0 C- 100 C
        // ros_bms_message_.batt_soc = GenerateRandomNumber(0, 100);       // 电量
        // ros_bms_message_.key_val = GenerateRandomNumber(0, 5);          // key_val
        // ros_bms_message_.batt_health = GenerateRandomNumber(0, 100);    // batt_health
        // ros_bms_message_.batt_loop_number = GenerateRandomNumber(0, 1000);  // batt_loop_number
        // ros_bms_message_.powerboard_status = GenerateRandomNumber(0, 6);    // powerboard_status
        std::this_thread::sleep_for(std::chrono::seconds(1));
      }
    });

  simulation_task.detach();
}

int BluetoothCarpo::GenerateRandomNumber(int start, int end)
{
  std::random_device rd;   // 将用于为随机数引擎获得种子
  std::mt19937 gen(rd());  // 以播种标准 mersenne_twister_engine
  std::uniform_int_distribution<> dis(start, end);  // [start end]
  return dis(gen);
}

}  //  namespace device
}  //  namespace cyberdog

PLUGINLIB_EXPORT_CLASS(cyberdog::device::BluetoothCarpo, cyberdog::device::BluetoothBase)
