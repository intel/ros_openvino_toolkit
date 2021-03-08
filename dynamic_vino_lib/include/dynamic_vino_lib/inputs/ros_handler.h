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
 * @brief A header file with declaration for ROS/ROS2 handler
 * @file ros_handler.h
 */

#ifndef DYNAMIC_VINO_LIB_INPUTS_ROS_HANDLER_H
#define DYNAMIC_VINO_LIB_INPUTS_ROS_HANDLER_H

#include <ros/ros.h>
#include <ros/types.h>
#include <ros/time.h>

namespace Input
{

class RosHandler
{
public:
  void setHandler(const std::shared_ptr<ros::NodeHandle>& node)
  {
    node_ = node;
  }

  std::shared_ptr<ros::NodeHandle> getHandler() const
  {
    return node_;
  }

  // inline void setHeader(std::string frame_id)
  // {
  //   header_.frame_id = frame_id;
  // #if true //directly use RCLCPP api for time stamp generation.
  //   header_.stamp = ros::Time::now();
  // #else
  //   std::chrono::high_resolution_clock::time_point tp = std::chrono::high_resolution_clock::now();
  //   int64 ns = tp.time_since_epoch().count();
  //   header_.stamp.sec = ns / 1000000000;
  //   header_.stamp.nanosec = ns % 1000000000;
  // #endif
  // }

  // inline void setHeader(std_msgs::Header header)
  // {
  //   header_ = header;
  // }

  // inline void lockHeader()
  // {
  //   locked_header_ = header_;
  // }

  // /**
  //  * @brief Get the frame_id of input device.
  //  * @return Frame_id of input device.
  //  */

  // inline std_msgs::Header getLockedHeader()
  // {
  //   return locked_header_;
  // }

private:
  std::shared_ptr<ros::NodeHandle> node_;

  // std_msgs::Header header_;
  // std_msgs::Header locked_header_;
};

}  // namespace Input

#endif  // DYNAMIC_VINO_LIB_INPUTS_ROS_HANDLER_H
