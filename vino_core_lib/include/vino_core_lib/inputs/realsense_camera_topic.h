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

#ifndef VINO_CORE_LIB__INPUTS__REALSENSE_CAMERA_TOPIC_H
#define VINO_CORE_LIB__INPUTS__REALSENSE_CAMERA_TOPIC_H

#include <image_transport/image_transport.h>
#include <nodelet/nodelet.h>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <opencv2/opencv.hpp>

#include <memory>

#include "vino_core_lib/inputs/base_input.h"
#include "vino_core_lib/inputs/image_topic.h"

namespace Input
{
/**
 * DEPRECATED!
 * Using the new class ImageTopic to handle all image topics.
 * @class RealSenseCameraTopic
 * @brief Class for recieving a realsense camera topic as input.
 */
typedef ImageTopic RealSenseCameraTopic;

}  // namespace Input

#endif  // VINO_CORE_LIB__INPUTS__REALSENSE_CAMERA_TOPIC_H_
