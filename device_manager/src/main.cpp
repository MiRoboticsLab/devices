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

#include <string>
#include <memory>

#include "device_manager/device_manager.hpp"
#include "cyberdog_common/cyberdog_log.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  LOGGER_MAIN_INSTANCE("device_manager");

  auto device_manager =
    std::make_shared<cyberdog::device::DeviceManager>(std::string("device_manager"));
  device_manager->Config();
  if (!device_manager->Init()) {
    ERROR("device manager init failed!");
    return -1;
  }
  device_manager->Run();
  return 0;
}
