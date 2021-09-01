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
 * @brief a header file with declaration of ImageTopic class
 * @file image_topic.cpp
 */

#include <cv_bridge/cv_bridge.h>
#include "vino_core_lib/inputs/image_topic.h"
#include "vino_core_lib/slog.h"

#define INPUT_TOPIC "/camera/color/image_raw"

bool Input::ImageTopic::initialize()
{
  slog::info << "before Image Topic init" << slog::endl;
  std::shared_ptr<image_transport::ImageTransport> it = std::make_shared<image_transport::ImageTransport>(nh_);
  sub_ = it->subscribe(INPUT_TOPIC, 1, &ImageTopic::cb, this);

  return true;
}

bool Input::ImageTopic::initialize(size_t width, size_t height)
{
  slog::warn << "BE CAREFUL: nothing for resolution is done when calling "
                "initialize(width, height)"
             << " for Image Topic" << slog::endl;
  return initialize();
}

void Input::ImageTopic::cb(const sensor_msgs::Image::ConstPtr& image_msg)
{
  slog::debug << "Receiving a new image from Camera topic." << slog::endl;
  // setHeader(image_msg->header);

  image_ = cv_bridge::toCvCopy(image_msg, "bgr8")->image;
  // Suppose Image Topic is sent within BGR order, so the below line would work.
  // image_ = cv::Mat(image_msg->height, image_msg->width, CV_8UC3,
  //  const_cast<uchar *>(&image_msg->data[0]), image_msg->step);

  image_count_.increaseCounter();
}

bool Input::ImageTopic::read(cv::Mat* frame)
{
  if (image_count_.get() < 0 || image_.empty())
  {
    slog::debug << "No data received in CameraTopic instance" << slog::endl;
    return false;
  }

  *frame = image_;
  // lockHeader();
  image_count_.decreaseCounter();
  return true;
}
