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

#ifndef CYBERDOG_PHOTO__PHOTO_BASE_HPP_
#define CYBERDOG_PHOTO__PHOTO_BASE_HPP_

#include <protocol/srv/take_photo.hpp>

#include <memory>
#include <functional>
#include "rclcpp/rclcpp.hpp"

namespace cyberdog
{
namespace device
{
class PhotoBase
{
public:
  virtual ~PhotoBase() {}
  virtual bool Init(bool simulation = false) = 0;
  std::function<void(std::shared_ptr<protocol::srv::TakePhoto::Response>)> TakePhoto;

protected:
  PhotoBase() {}
};
}  // namespace device
}  // namespace cyberdog

#endif  // CYBERDOG_PHOTO__PHOTO_BASE_HPP_
