// Copyright (c) 2018 Intel Corporation
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

/**
 * @brief A header file with declaration for ImageTopic class
 * @file image_topic.h
 */

#ifndef DYNAMIC_VINO_LIB__INPUTS__IMAGE_TOPIC_HPP_
#define DYNAMIC_VINO_LIB__INPUTS__IMAGE_TOPIC_HPP_

#include "dynamic_vino_lib/inputs/base_input.h"
#include "dynamic_vino_lib/utils/mutex_counter.hpp"
#include <condition_variable>
#include <image_transport/image_transport.h>
#include <memory>
#include <mutex>
#include <opencv2/opencv.hpp>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>

namespace Input {
/**
 * @class ImageTopic
 * @brief Class for recieving a realsense camera topic as input.
 */
class ImageTopic : public BaseInputDevice {
public:
  bool initialize() override;
  bool initialize(size_t width, size_t height) override;
  bool read(cv::Mat *frame) override;

private:
  ros::NodeHandle nh_;
  image_transport::Subscriber sub_;
  cv::Mat image_;
  MutexCounter image_count_;

  void cb(const sensor_msgs::ImageConstPtr &image_msg);
};
} // namespace Input

#endif // DYNAMIC_VINO_LIB__INPUTS__IMAGE_TOPIC_HPP_
