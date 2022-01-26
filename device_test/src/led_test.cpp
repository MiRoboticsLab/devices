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
#include <iostream>
#include <gtest/gtest.h>
#include "rclcpp/rclcpp.hpp"
#include "cyberdog_system/robot_code.hpp"
#include "protocol/srv/led_execute.hpp"
#include "protocol/msg/touch_status.hpp"


using LedSrv = protocol::srv::LedExecute;
using LedRequest = protocol::srv::LedExecute_Request;

TEST(LedTest, First)
{
  printf("hello\n");
  rclcpp::Node::SharedPtr node_ptr = std::make_shared<rclcpp::Node>("First");
  auto clinet = node_ptr->create_client<LedSrv>("led_execute");
  auto request = std::make_shared<LedRequest>();
  request->client = std::string("LedTester");
  request->priority = 5;
  request->target = 1;
  request->timeout = 1000;
  std::vector<int32_t> id_vec;
  id_vec.push_back(1);
  id_vec.push_back(2);
  id_vec.push_back(3);

  if(!clinet->wait_for_service(std::chrono::seconds(5))) {
    std::cout<<"fail to run the service"<<std::endl;
    return;
  }
  for(auto id : id_vec) {
    request->effect = id;
    auto result = clinet->async_send_request(request);
    if(rclcpp::spin_until_future_complete(node_ptr, result) != rclcpp::FutureReturnCode::SUCCESS) {
      return;
    } else {
      EXPECT_EQ(result.get()->code, (int32_t)cyberdog::system::KeyCode::kOK);
    }
    std::this_thread::sleep_for(std::chrono::seconds(2));
  }

}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);
  return RUN_ALL_TESTS();
}

