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
 * @brief A header file with declaration for RosTopicOutput Class
 * @file ros_topic_output.h
 */

#ifndef DYNAMIC_VINO_LIB__OUTPUTS__ROS_SERVICE_OUTPUT_HPP_
#define DYNAMIC_VINO_LIB__OUTPUTS__ROS_SERVICE_OUTPUT_HPP_

#include <object_msgs/DetectObject.h>
#include <object_msgs/Object.h>
#include <object_msgs/ObjectInBox.h>
#include <object_msgs/ObjectsInBoxes.h>
#include <people_msgs/AgeGender.h>
#include <people_msgs/AgeGenderStamped.h>
#include <people_msgs/Emotion.h>
#include <people_msgs/EmotionsStamped.h>
#include <people_msgs/HeadPose.h>
#include <people_msgs/HeadPoseStamped.h>
#include <people_msgs/ObjectInMask.h>
#include <people_msgs/ObjectsInMasks.h>
#include <people_msgs/Reidentification.h>
#include <people_msgs/ReidentificationStamped.h>

#include <object_msgs/DetectObjectRequest.h>
#include <people_msgs/AgeGenderSrv.h>
#include <people_msgs/EmotionSrv.h>
#include <people_msgs/HeadPoseSrv.h>
#include <people_msgs/ObjectsInMasksSrv.h>
#include <people_msgs/PeopleSrv.h>
#include <people_msgs/ReidentificationSrv.h>

#include <std_msgs/Header.h>

#include <memory>
#include <string>
#include <vector>

#include "dynamic_vino_lib/inferences/face_detection.h"
#include "dynamic_vino_lib/outputs/ros_topic_output.h"

namespace Outputs {
/**
 * @class RosServiceOutput
 * @brief This class handles and publish the detection result for service
 * calling.
 */
class RosServiceOutput : public RosTopicOutput {
public:
  RosServiceOutput(std::string pipeline_name) : pipeline_name_(pipeline_name) {}

  /**
   * @brief Publish all the detected infomations generated by the accept
   * functions with ros topic.
   */
  void handleOutput() override {}
  void clearData();

  void setServiceResponse(
      boost::shared_ptr<object_msgs::DetectObject::Response> response);
  void setResponseForFace(
      boost::shared_ptr<object_msgs::DetectObject::Response> response);
  void setServiceResponse(
      boost::shared_ptr<people_msgs::AgeGenderSrv::Response> response);
  void setServiceResponse(
      boost::shared_ptr<people_msgs::EmotionSrv::Response> response);
  void setServiceResponse(
      boost::shared_ptr<people_msgs::HeadPoseSrv::Response> response);
  void setServiceResponse(
      boost::shared_ptr<people_msgs::PeopleSrv::Response> response);
  void setServiceResponse(
      boost::shared_ptr<people_msgs::ObjectsInMasksSrv::Response> response);
  void setServiceResponse(
      boost::shared_ptr<people_msgs::ReidentificationSrv::Response> response);

private:
  const std::string service_name_;
  const std::string pipeline_name_;
};
} // namespace Outputs
#endif // DYNAMIC_VINO_LIB__OUTPUTS__ROS_SERVICE_OUTPUT_HPP_
