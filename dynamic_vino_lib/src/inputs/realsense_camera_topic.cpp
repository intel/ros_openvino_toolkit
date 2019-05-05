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
 * @brief a header file with declaration of RealSenseCamera class
 * @file realsense_camera_topic.cpp
 */

#include "dynamic_vino_lib/inputs/realsense_camera_topic.h"
#include <image_transport/image_transport.h>
#include "dynamic_vino_lib/slog.h"

#include <cv_bridge/cv_bridge.h>

#define INPUT_TOPIC "/camera/color/image_raw"

Input::RealSenseCameraTopic::RealSenseCameraTopic()
{
}

bool Input::RealSenseCameraTopic::initialize()
{
  slog::info << "before cameraTOpic init" << slog::endl;
  
  std::shared_ptr<image_transport::ImageTransport> it =
	        std::make_shared<image_transport::ImageTransport>(nh_);
  sub_ = it->subscribe("/camera/color/image_raw", 1, &RealSenseCameraTopic::cb,
		                           this); 
  return true;
}

void Input::RealSenseCameraTopic::cb(
    const sensor_msgs::ImageConstPtr& image_msg)
{
  image = cv_bridge::toCvCopy(image_msg, "bgr8")->image;
}

bool Input::RealSenseCameraTopic::read(cv::Mat* frame)
{
  ros::spinOnce();
  //nothing in topics from begining
  if (image.empty() && last_image.empty())
  {
    slog::warn << "No data received in CameraTopic instance" << slog::endl;
    return false;
  }
  if(image.empty()) {
	  *frame = last_image;
  }
  else
  {
	  *frame = image;
  }
  return true;
}

void Input::RealSenseCameraTopic::config()
{
  // TODO(weizhi): config
}
