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
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "device_manager/device_manager.hpp"
#include "ament_index_cpp/get_package_share_directory.hpp"

cyberdog::device::DeviceManager::DeviceManager(const std::string & name)
: cyberdog::machine::MachineActuator(name),
  name_(name)
{
  node_ptr = rclcpp::Node::make_shared(name_);
  executor.add_node(node_ptr);
  device_handler_ = std::make_shared<cyberdog::device::DeviceHandler>();
  heart_beats_ptr_ = std::make_unique<cyberdog::machine::HeartBeatsActuator>("device");
  code_ptr_ = std::make_shared<cyberdog::system::CyberdogCode<DeviceErrorCode>>(
    cyberdog::system::ModuleCode::kDeviceManager);
}

cyberdog::device::DeviceManager::~DeviceManager()
{}

void cyberdog::device::DeviceManager::Config()
{
  device_handler_->Config();
  // config priority and robot states
}

bool cyberdog::device::DeviceManager::Init()
{
  auto local_share_dir = ament_index_cpp::get_package_share_directory("params");
  auto path = local_share_dir + std::string("/toml_config/manager/state_machine_config.toml");
  if (!this->MachineActuatorInit(
      path,
      node_ptr))
  {
    ERROR("Init failed, actuator init error.");
    return false;
  }
  this->RegisterStateCallback(SetUp_V, std::bind(&DeviceManager::OnSetUp, this));
  this->RegisterStateCallback(TearDown_V, std::bind(&DeviceManager::ONTearDown, this));
  this->RegisterStateCallback(SelfCheck_V, std::bind(&DeviceManager::SelfCheck, this));
  this->RegisterStateCallback(Active_V, std::bind(&DeviceManager::OnActive, this));
  this->RegisterStateCallback(DeActive_V, std::bind(&DeviceManager::OnDeActive, this));
  this->RegisterStateCallback(Protected_V, std::bind(&DeviceManager::OnProtected, this));
  this->RegisterStateCallback(LowPower_V, std::bind(&DeviceManager::OnLowPower, this));
  this->RegisterStateCallback(OTA_V, std::bind(&DeviceManager::OnOTA, this));
  this->RegisterStateCallback(Error_V, std::bind(&DeviceManager::OnError, this));
  heart_beats_ptr_->HeartBeatRun();
  INFO("-------------------------------init:heart run and state actuator start");
  return this->ActuatorStart();
}

int32_t cyberdog::device::DeviceManager::SelfCheck()
{
  int32_t err_code = device_handler_->SelfCheck();
  if (err_code == 0) {
    return code_ptr_->GetKeyCode(cyberdog::system::KeyCode::kOK);
  } else {
    return err_code;
  }
}

void cyberdog::device::DeviceManager::Run()
{
  executor.spin();
  rclcpp::shutdown();
}

int32_t cyberdog::device::DeviceManager::OnError()
{
  ERROR("device on error");
  return code_ptr_->GetKeyCode(cyberdog::system::KeyCode::kOK);
}

int32_t cyberdog::device::DeviceManager::OnLowPower()
{
  ERROR("device on lowpower");
  return code_ptr_->GetKeyCode(cyberdog::system::KeyCode::kOK);
}

int32_t cyberdog::device::DeviceManager::OnSuspend()
{
  ERROR("device on suspend");
  return code_ptr_->GetKeyCode(cyberdog::system::KeyCode::kOK);
}

int32_t cyberdog::device::DeviceManager::OnProtected()
{
  ERROR("on protect");
  return code_ptr_->GetKeyCode(cyberdog::system::KeyCode::kOK);
}

int32_t cyberdog::device::DeviceManager::OnActive()
{
  INFO("device on active");
  if (!is_active) {
    callback_group_ =
      node_ptr->create_callback_group(rclcpp::CallbackGroupType::Reentrant);

    callback_group_led =
      node_ptr->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    led_service_ = node_ptr->create_service<protocol::srv::LedExecute>(
      "led_execute",
      std::bind(
        &DeviceManager::LedServiceCallback, this,
        std::placeholders::_1, std::placeholders::_2),
      rmw_qos_profile_services_default, callback_group_led);

    bms_service_ = node_ptr->create_service<protocol::srv::BmsCmd>(
      "bms_cmd",
      std::bind(
        &DeviceManager::BmsControlCallback, this,
        std::placeholders::_1, std::placeholders::_2),
      rmw_qos_profile_services_default, callback_group_);

    uwb_service_ = node_ptr->create_service<protocol::srv::GetUWBMacSessionID>(
      "get_uwb_mac_session_id",
      std::bind(
        &DeviceManager::UwbServiceCallback, this,
        std::placeholders::_1, std::placeholders::_2),
      rmw_qos_profile_services_default, callback_group_);
    is_active = true;
  }

  uwb_connection_state_ = node_ptr->create_subscription<std_msgs::msg::Bool>(
    "uwb_connected", 5,
    std::bind(&DeviceManager::UwbConnectedCallback, this, std::placeholders::_1));
  return code_ptr_->GetKeyCode(cyberdog::system::KeyCode::kOK);
}

int32_t cyberdog::device::DeviceManager::OnDeActive()
{
  ERROR("device on deactive");
  return code_ptr_->GetKeyCode(cyberdog::system::KeyCode::kOK);
}

int32_t cyberdog::device::DeviceManager::OnSetUp()
{
  INFO("device on setup");
  if (!device_handler_->Init(node_ptr)) {
    // error msg
    ERROR("device setup fail.");
    return code_ptr_->GetKeyCode(cyberdog::system::KeyCode::kFailed);
  }
  INFO("device setup success");
  return code_ptr_->GetKeyCode(cyberdog::system::KeyCode::kOK);
}

int32_t cyberdog::device::DeviceManager::ONTearDown()
{
  ERROR("device on teardown");
  return code_ptr_->GetKeyCode(cyberdog::system::KeyCode::kOK);
}

int32_t cyberdog::device::DeviceManager::OnOTA()
{
  ERROR("device on ota");
  return code_ptr_->GetKeyCode(cyberdog::system::KeyCode::kOK);
}

bool cyberdog::device::DeviceManager::IsStateInvalid()
{
  // check from config priority and current states
  return true;
}

void cyberdog::device::DeviceManager::LedServiceCallback(
  const protocol::srv::LedExecute_Request::SharedPtr request,
  protocol::srv::LedExecute_Response::SharedPtr response)
{
  if (!IsStateInvalid()) {
    response->code = (int32_t)system::KeyCode::kStateInvalid;
    return;
  }
  device_handler_->ExecuteLed(request, response);
}

void cyberdog::device::DeviceManager::BmsControlCallback(
  const protocol::srv::BmsCmd_Request::SharedPtr request,
  protocol::srv::BmsCmd_Response::SharedPtr response)
{
  device_handler_->ExecuteBmsControl(request, response);
}

void cyberdog::device::DeviceManager::UwbServiceCallback(
  const protocol::srv::GetUWBMacSessionID_Request::SharedPtr request,
  protocol::srv::GetUWBMacSessionID_Response::SharedPtr response)
{
  device_handler_->ExecuteUwb(request, response);
}

void cyberdog::device::DeviceManager::UwbConnectedCallback(
  const std_msgs::msg::Bool::SharedPtr msg)
{
  device_handler_->UwbConnectionSignal(msg);
}
