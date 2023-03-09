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

#include <memory>
#include <string>
#include <vector>
#include <limits>
#include <utility>
#include <tuple>

#include "cyberdog_uwb/uwb_plugin.hpp"
#include "cyberdog_uwb/float_comparisons.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "ament_index_cpp/get_package_share_directory.hpp"
#include "protocol/srv/get_uwb_mac_session_id.hpp"

namespace cyberdog
{
namespace device
{

const char * kConfigFile = "/toml_config/device/uwb_config.toml";
const char * kDefaultPath = "/toml_config/device/";
const int kDefaultUWB_Count = 4;
const int kDefaultRetryTimes = 3;

bool UWBCarpo::Config()
{
  return true;
}

bool UWBCarpo::Init(
  std::function<void(UwbSignleStatusMsg)>
  function_callback, bool simulation)
{
  simulation_ = simulation;
  ros_uwb_status_.data.resize(kDefaultUWB_Count);
  const SYS::ModuleCode kModuleCode = SYS::ModuleCode::kMiniLED;
  code_ = std::make_shared<SYS::CyberdogCode<UWB_Code>>(kModuleCode);
  if (!LoadUWBTomlConfig()) {
    ERROR("Load UWB config failed!");
    return false;
  }

  RegisterTopic(function_callback);

  if (simulation_) {
    simulator_thread_ = std::thread(std::bind(&UWBCarpo::SimulationThread, this));
    simulator_thread_.detach();
  } else {
    // make link for can
    if (!InitCAN_Com()) {
      INFO("Create can communication error.");
      return false;
    } else if (!Initialize()) {
      INFO("Enable initialize error.");
      return false;
    }
  }
  INFO("[UWBCarpo]: %s", "UWBCarpo initialize success.");
  return true;
}

int32_t UWBCarpo::SelfCheck()
{
  return code_->GetKeyCode(SYS::KeyCode::kOK);
}

bool UWBCarpo::LowPower()
{
  return true;
}

bool UWBCarpo::RegisterTopic(std::function<void(UwbSignleStatusMsg)> publisher)
{
  topic_pub_ = publisher;
  return true;
}

void UWBCarpo::Play(
  const std::shared_ptr<protocol::srv::GetUWBMacSessionID::Request> info_request,
  std::shared_ptr<protocol::srv::GetUWBMacSessionID::Response> info_response)
{
  info_response->session_id = uwb_config_.session_id;
  info_response->master = uwb_config_.controller_mac;
  info_response->slave1 = uwb_config_.uwb_list[0].mac;
  info_response->slave2 = uwb_config_.uwb_list[1].mac;
  info_response->slave3 = uwb_config_.uwb_list[2].mac;
  info_response->slave4 = uwb_config_.uwb_list[3].mac;

  std::thread do_open([this]() {
      this->Open();
    });
  do_open.detach();
}

void UWBCarpo::SetConnectedState(bool connected)
{
  static bool connect = false;
  if (connected) {
    INFO("UWB set connected");
  } else {
    INFO("UWB set disconnected");
    if (connect) {
      if (!Close()) {
        WARN("Close UWB operation failed!");
      } else {
        INFO("Close UWB operation succeeded");
      }
    }
  }
  connect = connected;
}

bool UWBCarpo::Open()
{
  // int32_t return_code = code_->GetKeyCode(SYS::KeyCode::kOK);
  bool status_ok = true;
  if (!simulation_) {
    for (auto & uwb : uwb_map_) {
      if (uwb.second->GetData()->data_received) {
        INFO("[%s] opened successfully", uwb.first.c_str());
        continue;
      }
      int retry = 0;
      bool single_status_ok = true;
      while (retry++ < kDefaultRetryTimes) {
        uwb.second->Operate("enable_on", std::vector<uint8_t>{});
        if (uwb.second->GetData()->enable_on_signal.WaitFor(1000)) {
          if (!uwb.second->GetData()->data_received) {
            ERROR(
              "[%s] opened failed,can not receive enable on ack ,time[%d]",
              uwb.first.c_str(), retry);
            single_status_ok = false;
          } else {
            single_status_ok = true;
            INFO("[%s] opened successfully", uwb.first.c_str());
            break;
          }
        } else {
          if (uwb.second->GetData()->enable_on_ack == 0) {
            single_status_ok = true;
            INFO("[%s] opened successfully", uwb.first.c_str());
            break;
          } else {
            single_status_ok = false;
            ERROR(
              "[%s] opened failed, get ack 0x%x!", uwb.first.c_str(),
              uwb.second->GetData()->enable_on_ack);
          }
        }
      }
      if (!single_status_ok) {status_ok = false;}
    }
  }
  // if (!status_ok) {return_code = code_->GetKeyCode(SYS::KeyCode::kFailed);}
  is_working_ = (status_ok ? true : false);
  return status_ok;
}

bool UWBCarpo::Close()
{
  // int32_t return_code = code_->GetKeyCode(SYS::KeyCode::kOK);
  bool status_ok = true;
  if (!simulation_) {
    for (auto & uwb : uwb_map_) {
      int retry = 0;
      bool single_status_ok = true;
      while (retry++ < kDefaultRetryTimes) {
        uwb.second->Operate("enable_off", std::vector<uint8_t>{});
        if (uwb.second->GetData()->enable_off_signal.WaitFor(1000)) {
          if (IsSingleClosed(uwb.first)) {
            INFO("[%s] stoped successfully", uwb.first.c_str());
            single_status_ok = true;
            break;
          }
          ERROR(
            "[%s] stoped failed,can not receive enable off ack,time[%d]",
            uwb.first.c_str(), retry);
          single_status_ok = false;
        } else {
          if (uwb.second->GetData()->enable_off_ack == 0) {
            if (CheckClosed(kDefaultRetryTimes, uwb.first)) {
              INFO("[%s] stoped successfully", uwb.first.c_str());
              single_status_ok = true;
              break;
            } else {
              ERROR(
                "[%s] stoped failed,get ack but data is receving,time[%d]",
                uwb.first.c_str(), retry);
              single_status_ok = false;
            }
          } else {
            single_status_ok = false;
            ERROR(
              "[%s] stoped failed, get ack 0x%x!", uwb.first.c_str(),
              uwb.second->GetData()->enable_off_ack);
          }
        }
      }
      if (!single_status_ok) {status_ok = false;}
    }
  }
  is_working_ = (status_ok ? false : true);
  // if (!status_ok) {return_code = code_->GetKeyCode(SYS::KeyCode::kFailed);}
  return status_ok;
}

bool UWBCarpo::Initialize()
{
  INFO("NOW UWBCarpo::Initialize");
  // get random UWBConnectInfo
  if (!uwb_config_.use_static_mac) {
    INFO("UWB use random mac.");
    uwb_config_.session_id = GenerateRandomNumber(0xFF, 0x7FFFFF00);
    uwb_config_.controller_mac = GenerateRandomNumber(0x00FF, 0x3F00);
    uint16_t start = 0x3F00;
    for (int i = 0; i < uwb_config_.uwb_list.size(); i++) {
      uwb_config_.uwb_list[i].mac = GenerateRandomNumber(start + 1, start + 0x3000);
      start += 0x3000;
    }
  }
  INFO(
    "session_id=%04x mac=%02x, %02x, %02x, %02x, %02x",
    uwb_config_.session_id,
    uwb_config_.controller_mac,
    uwb_config_.uwb_list[0].mac,
    uwb_config_.uwb_list[1].mac,
    uwb_config_.uwb_list[2].mac,
    uwb_config_.uwb_list[3].mac);

  auto InitUwb = [&]() {
      bool status_ok = true;
      uint8_t buf[8] = {0};
      memcpy(&buf[0], &uwb_config_.session_id, sizeof(uwb_config_.session_id));
      memcpy(&buf[4], &uwb_config_.controller_mac, sizeof(uwb_config_.controller_mac));
      std::vector<uint8_t> Cmd(buf, buf + sizeof(buf) / sizeof(buf[0]));
      if (!simulation_) {
        for (auto & uwb : uwb_map_) {
          int retry = 0;
          bool single_status_ok = true;
          uint16_t mac = uwb_config_.uwb_list[uwb.second->GetData()->index].mac;
          Cmd[6] = mac & 0xFF;
          Cmd[7] = (mac >> 8) & 0xFF;
          while (retry++ < kDefaultRetryTimes) {
            uwb.second->Operate("enable_initial", Cmd);
            if (uwb.second->GetData()->enable_initial_signal.WaitFor(1000)) {
              ERROR(
                "[%s] opened failed,can not receive enable init ack ,time[%d]",
                uwb.first.c_str(), retry);
              single_status_ok = false;
            } else {
              if (uwb.second->GetData()->enable_initial_ack == 0) {
                single_status_ok = true;
                INFO("[%s] inited successfully", uwb.first.c_str());
                break;
              } else {
                single_status_ok = false;
                ERROR(
                  "[%s] inited failed, get ack 0x%x!", uwb.first.c_str(),
                  uwb.second->GetData()->enable_initial_ack);
              }
            }
          }
          if (!single_status_ok) {status_ok = false;}
        }
      }
      if (status_ok) {
        INFO("UWB initialized successfully");
      } else {
        INFO("UWB initialized failed");
      }
      return status_ok;
    };

  return InitUwb();
}

void UWBCarpo::UWB_MsgCallback(EP::DataLabel & label, std::shared_ptr<UWB_Msg> data)
{
  const int kMsgCheckInterval = 200;   // ms
  const int kMsgLogInterval = 3000;   // ms
  auto MsgCheck = [&](int index) {
      static Clock::time_point start_time = Clock::now();
      static Clock::time_point start_log = Clock::now();
      static uint8_t flag = 0;
      auto now = Clock::now();
      auto duration =
        std::chrono::duration_cast<std::chrono::milliseconds>(
        now - start_time);
      if (duration.count() >= kMsgCheckInterval) {
        flag = 0;
        start_time = Clock::now();
      }
      flag |= (1 << index);
      if (flag == 0xF) {
        auto duration =
          std::chrono::duration_cast<std::chrono::milliseconds>(
          now - start_log);
        if (duration.count() >= kMsgLogInterval) {
          for (auto & uwb : uwb_map_) {
            WARN(
              "name:[%s],dist:[%f],angle:[%f],nLos[%f],rssi_1[%f],rssi_2[%f]",
              uwb.first.c_str(),
              ros_uwb_status_.data[uwb.second->GetData()->index].dist,
              ros_uwb_status_.data[uwb.second->GetData()->index].angle,
              ros_uwb_status_.data[uwb.second->GetData()->index].n_los,
              ros_uwb_status_.data[uwb.second->GetData()->index].rssi_1,
              ros_uwb_status_.data[uwb.second->GetData()->index].rssi_2
            );
          }
          start_log = Clock::now();
        }
        TryPublish();
        flag = 0;
        start_time = Clock::now();
      }
    };
  if (uwb_map_.find(label.group_name) != uwb_map_.end()) {
    if (label.name == "enable_initial_ack") {
      if (data->enable_initial_ack != 0) {
        ERROR(
          "%s,enable initial ack err 0x:%x", label.group_name.c_str(),
          data->enable_initial_ack);
      }
      uwb_map_.at(label.group_name)->GetData()->enable_initial_signal.Give();
    } else if (label.name == "enable_on_ack") {
      if (data->enable_on_ack != 0) {
        ERROR("%s,enable on ack err 0x:%x", label.group_name.c_str(), data->enable_on_ack);
      }
      uwb_map_.at(label.group_name)->GetData()->enable_on_signal.Give();
      data_flag_ = 0;
    } else if (label.name == "enable_off_ack") {
      if (data->enable_off_ack != 0) {
        ERROR("%s,enable off ack err 0x:%x", label.group_name.c_str(), data->enable_off_ack);
      }
      uwb_map_.at(label.group_name)->GetData()->enable_off_signal.Give();
    } else if (label.name == "data") {
      if (uwb_map_.at(label.group_name)->GetData()->waiting_data) {
        uwb_map_.at(label.group_name)->GetData()->data_signal.Give();
      }
      if (!uwb_map_.at(label.group_name)->GetData()->data_received) {
        uwb_map_.at(label.group_name)->GetData()->data_received = true;
      }

      if (label.is_full) {
        if (data->index < kDefaultUWB_Count) {
          if (data->distance != 0) {
            ros_uwb_status_.data[data->index].dist = static_cast<float>(data->distance) / 100.0;
            ros_uwb_status_.data[data->index].angle = DegToRad(format_9_7(data->angle));
            ros_uwb_status_.data[data->index].n_los = data->nLos;
            ros_uwb_status_.data[data->index].rssi_1 = format_8_8(data->rssi_1);
            ros_uwb_status_.data[data->index].rssi_2 = format_8_8(data->rssi_2);
            MsgCheck(data->index);
          } else {
            WARN("    group_name ==   %s,receved data error 0", label.group_name.c_str());
          }
        } else {
          ERROR("%s get date index [%d] out of range", label.group_name.c_str(), data->index);
        }
      } else {
        WARN("%s get error data", label.group_name.c_str());
      }
    } else {
      WARN("data name %s no handle!", label.name.c_str());
    }
  } else {
    ERROR("can drive error,unknown name %s", label.group_name.c_str());
  }
}

bool UWBCarpo::InitCAN_Com()
{
  auto local_share_dir = ament_index_cpp::get_package_share_directory("params");
  for (auto & uwb : uwb_config_.uwb_list) {
    auto path = local_share_dir + kDefaultPath + uwb.com_file;
    std::shared_ptr<EP::Protocol<UWB_Msg>> uwb_msg = std::make_shared<EP::Protocol<UWB_Msg>>(
      path, false);
    // bind mac and index
    uwb_msg->GetData()->mac = uwb.mac;
    uwb_msg->GetData()->index = uwb.index;
    if (uwb_map_.find(uwb_msg->GetName()) == uwb_map_.end()) {
      uwb_map_.insert(std::make_pair(uwb_msg->GetName(), uwb_msg));
      uwb_msg->GetData()->data_received = false;
      uwb_msg->GetData()->waiting_data = false;
      uwb_msg->LINK_VAR(uwb_msg->GetData()->enable_initial_ack);
      uwb_msg->LINK_VAR(uwb_msg->GetData()->enable_on_ack);
      uwb_msg->LINK_VAR(uwb_msg->GetData()->enable_off_ack);
      uwb_msg->LINK_VAR(uwb_msg->GetData()->data);
      uwb_msg->SetDataCallback(
        std::bind(
          &UWBCarpo::UWB_MsgCallback, this, std::placeholders::_1, std::placeholders::_2));
    }
  }
  return true;
}

void UWBCarpo::SimulationThread()
{
  while (1) {
    struct timespec ts;
    clock_gettime(CLOCK_REALTIME, &ts);
    for (int i = 0; i < kDefaultUWB_Count; i++) {
      GenerateRandomNumber(0, 100);
      ros_uwb_status_.data[i].header.frame_id = "uwb_sensor";
      ros_uwb_status_.data[i].header.stamp.nanosec = ts.tv_nsec;
      ros_uwb_status_.data[i].header.stamp.sec = ts.tv_sec;
      ros_uwb_status_.data[i].dist = GenerateRandomNumber(0, 100);
      ros_uwb_status_.data[i].angle = GenerateRandomNumber(0, 100);
      ros_uwb_status_.data[i].n_los = GenerateRandomNumber(0, 100);
      ros_uwb_status_.data[i].rssi_1 = GenerateRandomNumber(0, 100);
      ros_uwb_status_.data[i].rssi_1 = GenerateRandomNumber(0, 100);
    }
    if (is_working_) {
      TryPublish();
    }
    std::this_thread::sleep_for(std::chrono::seconds(1));
  }
}

int UWBCarpo::GenerateRandomNumber(int start, int end)
{
  std::random_device rd;   // 将用于为随机数引擎获得种子
  std::mt19937 gen(rd());  // 以播种标准 mersenne_twister_engine
  std::uniform_int_distribution<> dis(start, end);  // [start end]
  return dis(gen);
}

bool UWBCarpo::LoadUWBTomlConfig()
{
  toml::value params_toml;
  toml::value config_files;
  auto local_share_dir = ament_index_cpp::get_package_share_directory("params");
  auto path = local_share_dir + kConfigFile;

  if (!TomlParse::ParseFile(path.c_str(), params_toml)) {
    ERROR("Params config file is not in toml format");
    return false;
  }

  if (!TomlParse::Get(params_toml, "front_back_threshold", uwb_config_.front_back_threshold)) {
    ERROR("toml file[%s] get [front_back_threshold] failed!", path.c_str());
    return false;
  }
  if (!TomlParse::Get(params_toml, "left_right_threshold", uwb_config_.left_right_threshold)) {
    ERROR("toml file[%s] get [left_right_threshold] failed!", path.c_str());
    return false;
  }

  // simulate
  bool simulate;
  if (TomlParse::Get(params_toml, "simulate", simulate)) {
    simulation_ = simulate;
    INFO("toml file[%s] get [simulate] change mode to sim:%d!", path.c_str(), simulation_);
  }

  // threshold
  if (!TomlParse::Get(params_toml, "AoA_F_NMAX", uwb_config_.AoA_F_NMAX)) {
    ERROR("toml file[%s] get [AoA_F_NMAX] failed!", path.c_str());
    return false;
  }
  if (!TomlParse::Get(params_toml, "AoA_F_PMAX", uwb_config_.AoA_F_PMAX)) {
    ERROR("toml file[%s] get [AoA_F_PMAX] failed!", path.c_str());
    return false;
  }
  if (!TomlParse::Get(params_toml, "AoA_B_NMAX", uwb_config_.AoA_B_NMAX)) {
    ERROR("toml file[%s] get [AoA_B_NMAX] failed!", path.c_str());
    return false;
  }
  if (!TomlParse::Get(params_toml, "AoA_B_PMAX", uwb_config_.AoA_B_PMAX)) {
    ERROR("toml file[%s] get [AoA_B_PMAX] failed!", path.c_str());
    return false;
  }
  if (!TomlParse::Get(params_toml, "AoA_L_NMAX", uwb_config_.AoA_L_NMAX)) {
    ERROR("toml file[%s] get [AoA_L_NMAX] failed!", path.c_str());
    return false;
  }
  if (!TomlParse::Get(params_toml, "AoA_L_PMAX", uwb_config_.AoA_L_PMAX)) {
    ERROR("toml file[%s] get [AoA_L_PMAX] failed!", path.c_str());
    return false;
  }
  if (!TomlParse::Get(params_toml, "AoA_R_NMAX", uwb_config_.AoA_R_NMAX)) {
    ERROR("toml file[%s] get [AoA_R_NMAX] failed!", path.c_str());
    return false;
  }
  if (!TomlParse::Get(params_toml, "AoA_R_PMAX", uwb_config_.AoA_R_PMAX)) {
    ERROR("toml file[%s] get [AoA_R_PMAX] failed!", path.c_str());
    return false;
  }
  if (!TomlParse::Get(params_toml, "use_static_mac", uwb_config_.use_static_mac)) {
    ERROR("toml file[%s] get [use_static_mac] failed!", path.c_str());
    return false;
  }
  if (!TomlParse::Get(params_toml, "session_id", uwb_config_.session_id)) {
    ERROR("toml file[%s] get [session_id] failed!", path.c_str());
    return false;
  }
  if (!TomlParse::Get(params_toml, "controller_mac", uwb_config_.controller_mac)) {
    ERROR("toml file[%s] get [controller_mac] failed!", path.c_str());
    return false;
  }
  // uwb cell config
  std::vector<toml::table> uwb_list;
  if (!TomlParse::Get(params_toml, "UWB", uwb_list)) {
    ERROR("toml file[%s] get [UWB] failed!", path.c_str());
    return false;
  }
  for (auto & uwb : uwb_list) {
    UWB_CellCfg cell_cfg;
    if (!TomlParse::Get(uwb, "name", cell_cfg.name)) {
      WARN("toml file[%s] [UWB]get [name] failed!", path.c_str());
      continue;
    }
    if (!TomlParse::Get(uwb, "com_file", cell_cfg.com_file)) {
      WARN("toml file[%s] [UWB]get [com_file] failed!", path.c_str());
      continue;
    }
    if (!TomlParse::Get(uwb, "mac", cell_cfg.mac)) {
      WARN("toml file[%s] [UWB]get [mac] failed!", path.c_str());
      continue;
    }
    if (!TomlParse::Get(uwb, "index", cell_cfg.index)) {
      WARN("toml file[%s] [UWB]get [index] failed!", path.c_str());
      continue;
    }
    uwb_config_.uwb_list.push_back(cell_cfg);
  }
  if (uwb_config_.uwb_list.size() != kDefaultUWB_Count) {
    ERROR("toml file[%s] get [UWB] size:%d not eq 4!", path.c_str(), uwb_config_.uwb_list.size());
    return false;
  }

  INFO("[%s] success!", __func__);

  return true;
}

bool UWBCarpo::TryPublish()
{
  static UwbSignleStatusMsg uwb_pre;

  UwbSignleStatusMsg ros_msg;
  constexpr auto eps = std::numeric_limits<float>::epsilon();

  auto & uwb_front = ros_uwb_status_.data[static_cast<int>(Type::HeadTOF)];
  auto & uwb_back = ros_uwb_status_.data[static_cast<int>(Type::HeadUWB)];
  auto & uwb_left = ros_uwb_status_.data[static_cast<int>(Type::RearUWB)];
  auto & uwb_right = ros_uwb_status_.data[static_cast<int>(Type::RearTOF)];
  ros_msg = uwb_front;
  ros_msg.header.frame_id = "head_tof";
  if (ros_msg.rssi_1 > -1 || ros_msg.rssi_2 > -1) {
    INFO("uwb front rssi_1[%d] rssi_2[%d]", ros_msg.rssi_1, ros_msg.rssi_2);
    return false;
  }
  if (ros_msg.rssi_1 < uwb_back.rssi_1) {
    ros_msg = uwb_back;
    ros_msg.header.frame_id = "head_uwb";
  }
  if (ros_msg.rssi_1 < uwb_left.rssi_1) {
    ros_msg = uwb_left;
    ros_msg.header.frame_id = "rear_uwb";
  }
  if (ros_msg.rssi_1 < uwb_right.rssi_1) {
    ros_msg = uwb_right;
    ros_msg.header.frame_id = "rear_tof";
  }

  if (uwb_pre.rssi_1 > -1 || uwb_pre.rssi_2 > -1) {
    uwb_pre = ros_msg;
    INFO("uwb_pre init rssi_1[%d] rssi_2[%d]", uwb_pre.rssi_1, uwb_pre.rssi_2);
  } else {
    if ((uwb_pre.header.frame_id == "head_tof" &&
      ros_msg.header.frame_id != "head_uwb") ||
      (uwb_pre.header.frame_id == "head_uwb" &&
      ros_msg.header.frame_id != "head_tof") ||
      (uwb_pre.header.frame_id == "rear_uwb" &&
      ros_msg.header.frame_id != "rear_tof") ||
      (uwb_pre.header.frame_id == "rear_tof" &&
      ros_msg.header.frame_id != "rear_uwb"))
    {
      // 过滤间隔标签跳转
      uwb_pre = ros_msg;
    } else {
      ros_msg = uwb_pre;
    }
  }

  struct timespec time_stu;
  clock_gettime(CLOCK_REALTIME, &time_stu);
  ros_msg.header.stamp.nanosec = time_stu.tv_nsec;
  ros_msg.header.stamp.sec = time_stu.tv_sec;
  topic_pub_(ros_msg);
  return true;
}

bool UWBCarpo::CheckClosed(int times, const std::string & name)
{
  bool is_closed = false;
  while (times-- > 0) {
    is_closed = IsSingleClosed(name);
    if (is_closed) {break;}
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }
  uwb_map_.at(name)->GetData()->data_received = (!is_closed);
  return is_closed;
}

bool UWBCarpo::IsSingleClosed(const std::string & name)
{
  if (uwb_map_.find(name) == uwb_map_.end()) {
    INFO("uwb map not find [%s]", name.c_str());
    return false;
  }
  uwb_map_.at(name)->GetData()->waiting_data = true;
  bool is_closed = uwb_map_.at(name)->GetData()->data_signal.WaitFor(200) ? true : false;
  uwb_map_.at(name)->GetData()->waiting_data = false;
  return is_closed;
}

bool UWBCarpo::IsSingleStarted(const std::string & name)
{
  if (uwb_map_.find(name) == uwb_map_.end()) {
    INFO("uwb map not find [%s]", name.c_str());
    return false;
  }
  uwb_map_.at(name)->GetData()->waiting_data = true;
  bool is_started = uwb_map_.at(name)->GetData()->data_signal.WaitFor(1000) ? false : true;
  uwb_map_.at(name)->GetData()->waiting_data = false;
  return is_started;
}
}  //  namespace device
}  //  namespace cyberdog

PLUGINLIB_EXPORT_CLASS(cyberdog::device::UWBCarpo, cyberdog::device::UWBBase)
