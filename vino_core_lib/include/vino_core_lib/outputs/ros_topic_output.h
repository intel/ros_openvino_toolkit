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
 * @brief A header file with declaration for RosTopicOutput Class
 * @file ros_topic_output.h
 */

#ifndef VINO_CORE_LIB_OUTPUTS_ROS_TOPIC_OUTPUT_H
#define VINO_CORE_LIB_OUTPUTS_ROS_TOPIC_OUTPUT_H

#include <image_transport/image_transport.h>
#include <object_msgs/Object.h>
#include <object_msgs/ObjectInBox.h>
#include <object_msgs/ObjectsInBoxes.h>
#include <vino_people_msgs/AgeGender.h>
#include <vino_people_msgs/AgeGenderStamped.h>
#include <vino_people_msgs/Emotion.h>
#include <vino_people_msgs/EmotionsStamped.h>
#include <vino_people_msgs/HeadPose.h>
#include <vino_people_msgs/HeadPoseStamped.h>
#include <vino_people_msgs/ObjectInMask.h>
#include <vino_people_msgs/ObjectsInMasks.h>
#include <vino_people_msgs/Reidentification.h>
#include <vino_people_msgs/ReidentificationStamped.h>
#include <vino_people_msgs/HumanPose.h>
#include <vino_people_msgs/HumanPoseStamped.h>
#include <ros/ros.h>

#include <memory>
#include <string>
#include <vector>

#include "vino_core_lib/inferences/face_detection.h"
#include "vino_core_lib/outputs/base_output.h"

namespace Outputs
{
/**
 * @class RosTopicOutput
 * @brief This class handles and publish the detection result with ros topic.
 */
class RosTopicOutput : public BaseOutput
{
 public:
  RosTopicOutput();
  /**
   * @brief Calculate the camera matrix of a frame.
   * @param[in] A frame.
   */
  void feedFrame(const cv::Mat&) override;
  /**
   * @brief Publish all the detected infomations generated by the accept
   * functions with ros topic.
   */
  void handleOutput() override;
  /**
   * @brief Generate ros topic infomation according to
   * the person reidentification result.
   * @param[in] results a bundle of person reidentification results.
   */
  void accept(const std::vector<vino_core_lib::PersonReidentificationResult> &) override;
  /**
   * @brief Generate ros topic infomation according to
   * the object segmentation result.
   * @param[in] results a bundle of object segmentation results.
   */
  void accept(const std::vector<vino_core_lib::ObjectSegmentationResult> &) override;
  /**
   * @brief Generate ros topic infomation according to
   * the face detection result.
   * @param[in] An face detection result objetc.
   */
  void accept(
      const std::vector<vino_core_lib::FaceDetectionResult>&) override;
  /**
   * @brief Generate ros topic infomation according to
   * the emotion detection result.
   * @param[in] An emotion detection result objetc.
   */
  void accept(const std::vector<vino_core_lib::EmotionsResult>&) override;
  /**
   * @brief Generate ros topic infomation according to
   * the age gender detection result.
   * @param[in] An age gender detection result objetc.
   */
  void accept(const std::vector<vino_core_lib::AgeGenderResult> &) override;
  /**detected_objects_topic_
   * @brief Generate ros topic infomation according to
   * the headpose detection result.
   * @param[in] An head pose detection result objetc.
   */
  void accept(const std::vector<vino_core_lib::HeadPoseResult>&) override;
  /**
   * @brief Generate ros topic infomation according to
   * the headpose detection result.
   * @param[in] An head pose detection result objetc.
   */
  void accept(const std::vector<vino_core_lib::ObjectDetectionResult>&) override;

  /**
   * @brief Generate ros topic infomation according to
   * the human pose estimation result.
   * @param[in] An human pose estimation result objetc.
   */
  void accept(const std::vector<vino_core_lib::HumanPoseResult>&) override;


 private:
  std_msgs::Header getHeader();
  const std::string topic_name_;
  cv::Mat frame_;
  ros::NodeHandle nh_;
 protected:
  ros::Publisher pub_face_;
  std::shared_ptr<object_msgs::ObjectsInBoxes> faces_msg_ptr_;
  ros::Publisher pub_emotion_;
  std::shared_ptr<vino_people_msgs::EmotionsStamped> emotions_msg_ptr_;
  ros::Publisher pub_age_gender_;
  std::shared_ptr<vino_people_msgs::AgeGenderStamped> age_gender_msg_ptr_;
  ros::Publisher pub_headpose_;
  std::shared_ptr<vino_people_msgs::HeadPoseStamped> headpose_msg_ptr_;
  ros::Publisher pub_object_;
  std::shared_ptr<object_msgs::ObjectsInBoxes> object_msg_ptr_;
  ros::Publisher pub_person_reid_;
  std::shared_ptr<vino_people_msgs::ReidentificationStamped> person_reid_msg_ptr_;
  ros::Publisher pub_segmented_object_;
  std::shared_ptr<vino_people_msgs::ObjectsInMasks> segmented_object_msg_ptr_;
  ros::Publisher pub_human_pose_;
  std::shared_ptr<vino_people_msgs::HumanPoseStamped> human_pose_msg_ptr_;

};
}  // namespace Outputs
#endif  // VINO_CORE_LIB_OUTPUTS_ROS_TOPIC_OUTPUT_H
