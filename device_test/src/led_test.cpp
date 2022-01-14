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
#include <string>
#include <chrono>
#include <thread>
#include <gtest/gtest.h>
#include "rclcpp/rclcpp.hpp"
#include "protocol/srv/led_execute.hpp"
#include "protocol/msg/touch_status.hpp"

using LedSrv = protocol::srv::LedExecute;
using TouchMsg = protocol::msg::TouchStatus;

TEST(LedTest, First)
{
  printf("hello\n");
  rclcpp::Node::SharedPtr node_ptr = std::make_shared<rclcpp::Node>("First");
  auto _pub = node_ptr->create_publisher<TouchMsg>("touch_status", 10);
  TouchMsg msg;
  msg.touch_state = 1;
  msg.timestamp = 0;
  int counter = 0;
  while(counter < 10) {
    // printf("publish once: %d", counter++);
     _pub->publish(msg);
    std::this_thread::sleep_for(std::chrono::seconds(2));
  }
 
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);
  return RUN_ALL_TESTS();
}

