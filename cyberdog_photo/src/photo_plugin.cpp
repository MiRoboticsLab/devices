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

#include <opencv2/opencv.hpp>
#include <cyberdog_photo/photo_plugin.hpp>

namespace cyberdog
{
namespace device
{
std::string getTegraPipeline(int width, int height, int fps=60) {

  return  std::string("nvarguscamerasrc ! ") +
          std::string("video/x-raw(memory:NVMM), ") +
          std::string("width=(int)") + std::to_string(width) + ", " +
          std::string("height=(int)") + std::to_string(height)  + ", " +
          std::string("format=(string)NV12, ") +
          std::string("framerate=(fraction)") + std::to_string(fps) + "/1 ! " +
          std::string("nvvidconv flip-method=0 ! ") +
          std::string("video/x-raw, ") +
          std::string("format=(string)BGRx ! ") +
          std::string("videoconvert ! ") +
          std::string("video/x-raw, ") +
          std::string("format=(string)BGR ! ") +
          std::string("appsink");
}

std::string mat_type2encoding(int mat_type)
{
  switch (mat_type) {
    case CV_8UC1:
      return "mono8";
    case CV_8UC3:
      return "bgr8";
    case CV_16SC1:
      return "mono16";
    case CV_8UC4:
      return "rgba8";
    default:
      throw std::runtime_error("unsupported encoding type");
  }
}

bool PhotoCarpo::Init(bool simulation)
{
  auto default_fun = [](std::shared_ptr<protocol::srv::TakePhoto::Response> response)->void
  {
    response->message = "simulation mode";
    response->result = false;
    return;
  };
  if (simulation) {
    TakePhoto = default_fun;
  }
  else {
    TakePhoto =  std::bind(&cyberdog::device::PhotoCarpo::takePhoto, this, std::placeholders::_1);
  }
  return true;
}

void PhotoCarpo::takePhoto(std::shared_ptr<protocol::srv::TakePhoto::Response> response)
{
  cv::Mat image;
  cv::VideoCapture video_capture;
  INFO("Start open camera");
  if (!video_capture.open(getTegraPipeline(4208, 3120, 30), cv::CAP_GSTREAMER)) {
    ERROR("Unable to open camera");
    response->result = false;
    response->message = "Unable to open camera";
    return;
  }

  for (int i = 0; i < 30; ++i) {
    if (video_capture.read(image) && !image.empty() && i == 29) {
      INFO("Captured an image");
      //cv::imwrite("/home/mi/test.jpg", image);
      try {
        response->img.encoding = mat_type2encoding(image.type());
        response->result = true;
        response->message = "got an image!";
        response->img.header.frame_id = "camera";
        response->img.height = image.rows;
        response->img.width = image.cols;
        response->img.is_bigendian = false;
        response->img.step = static_cast<sensor_msgs::msg::Image::_step_type>(image.step);
        response->img.data.assign(image.datastart, image.dataend);
        INFO("Sending photo");
      } catch (const std::runtime_error& e) {
        response->result = false;
        response->message = "Unsupported encoding type";
        ERROR("Unsupported encoding type");
      }
    }
  }
  video_capture.release();
  return;
}
}
}

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(cyberdog::device::PhotoCarpo, cyberdog::device::PhotoBase)