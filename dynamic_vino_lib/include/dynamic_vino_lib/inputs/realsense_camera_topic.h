/*
 * Copyright (c) 2018 Intel Corporation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

/**
 * @brief A header file with declaration for RealSenseCamera class
 * @file realsense_camera.h
 */

#ifndef DYNAMIC_VINO_LIB_INPUTS_REALSENSE_CAMERA_TOPIC_H
#define DYNAMIC_VINO_LIB_INPUTS_REALSENSE_CAMERA_TOPIC_H

#include <image_transport/image_transport.h>
#include <nodelet/nodelet.h>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <opencv2/opencv.hpp>

#include <memory>

#include "dynamic_vino_lib/inputs/base_input.h"

namespace Input
{
/**
 * @class RealSenseCameraTopic
 * @brief Class for recieving a realsense camera topic as input.
 */
class RealSenseCameraTopic : public BaseInputDevice
{
 public:
  RealSenseCameraTopic();
  bool initialize() override;
  bool initialize(int t) override
  {
    return true;
  };
  bool initialize(size_t width, size_t height) override
  {
    return true;
  };
  bool read(cv::Mat* frame) override;
  void config() override;

 private:
  ros::NodeHandle nh_;
  image_transport::Subscriber sub_;
  cv::Mat image;
  cv::Mat last_image;

  void cb(const sensor_msgs::ImageConstPtr& image_msg);
};
}  // namespace Input

#endif  // DYNAMIC_VINO_LIB_INPUTS_REALSENSE_CAMERA_TOPIC_H_
