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
 * @brief A header file with declaration for HeadPoseDetectionModel Class
 * @file head_pose_detection_model.h
 */

#ifndef VINO_CORE_LIB_OUTPUTS_BASE_OUTPUT_H
#define VINO_CORE_LIB_OUTPUTS_BASE_OUTPUT_H

#include <string>
#include <vector>

#include "vino_core_lib/inferences/age_gender_detection.h"
#include "vino_core_lib/inferences/base_inference.h"
#include "vino_core_lib/inferences/emotions_detection.h"
#include "vino_core_lib/inferences/face_detection.h"
#include "vino_core_lib/inferences/head_pose_detection.h"
#include "vino_core_lib/inferences/object_detection.h"
#include "vino_core_lib/inferences/object_segmentation.h"
#include "vino_core_lib/inferences/person_reidentification.h"
#include "vino_core_lib/inferences/human_pose_estimation.h"
#include "vino_core_lib/services/frame_processing_server.h"
#include "opencv2/opencv.hpp"

class Pipeline;
namespace Outputs
{
/**
 * @class BaseOutput
 * @brief This class is a base class for various output devices. It employs
 * visitor pattern to perform different operations to different inference
 * result with different output device
 */
class BaseOutput
{
 public:
  // BaseOutput() = default;
  BaseOutput()
  {
  };
  /**
  * @brief Generate output content according to the face detection result.
  */
  virtual void accept(
      const std::vector<vino_core_lib::ObjectDetectionResult>&) 
  {
  }
  /**
   * @brief Generate output content according to the face detection result.
   */
  virtual void accept(
      const std::vector<vino_core_lib::FaceDetectionResult>&)
  {
  }
  /**
   * @brief Generate output content according to the emotion detection result.
   */
  virtual void accept(const std::vector<vino_core_lib::EmotionsResult>&)
  {
  }
  /**
   * @brief Generate output content according to the age and gender detection
   * result.
   */
  virtual void accept(const std::vector<vino_core_lib::AgeGenderResult>&)
  {
  }
  /**
   * @brief Generate output content according to the headpose detection result.
   */
  virtual void accept(const std::vector<vino_core_lib::HeadPoseResult>&)
  {
  }
  /**
   * @brief Generate output content according to the object segmentation result.
   */
  virtual void accept(const std::vector<vino_core_lib::ObjectSegmentationResult>&)
  {
  }
  /**
  * @brief Generate output content according to the person reidentification result.
  */
  virtual void accept(const std::vector<vino_core_lib::PersonReidentificationResult> &)
  {
  }
  /**
  * @brief Generate output content according to the human pose estimation result.
  */
  virtual void accept(const std::vector<vino_core_lib::HumanPoseResult> &)
  {
  }
  /**
   * @brief Calculate the camera matrix of a frame for image window output, no
         implementation for ros topic output.
   */
  virtual void feedFrame(const cv::Mat& frame)
  {
  }
  /**
   * @brief Show all the contents generated by the accept functions.
   */
  virtual void handleOutput() = 0;

  void setPipeline(Pipeline* const pipeline);
  virtual void setServiceResponse(
    boost::shared_ptr<object_msgs::DetectObjectResponse> response) {}
  virtual void setServiceResponseForFace(
    boost::shared_ptr<object_msgs::DetectObjectResponse> response) {}
  virtual void setServiceResponse(
    boost::shared_ptr<vino_people_msgs::AgeGenderSrvResponse> response) {}
  virtual void setServiceResponse(
    boost::shared_ptr<vino_people_msgs::EmotionSrvResponse> response) {}
  virtual void setServiceResponse(
    boost::shared_ptr<vino_people_msgs::HeadPoseSrvResponse> response) {}
  virtual void setServiceResponse(
    boost::shared_ptr<vino_people_msgs::PeopleSrvResponse> response) {}
  virtual void setServiceResponse(
    boost::shared_ptr<vino_people_msgs::ReidentificationSrvResponse> response) {}
  virtual void setServiceResponse(
    boost::shared_ptr<vino_people_msgs::ObjectsInMasksSrvResponse> response) {}
  Pipeline* getPipeline() const;
  cv::Mat getFrame() const;
  virtual void clearData() {}

 protected:
  cv::Mat frame_;
  Pipeline* pipeline_;
};
}  // namespace Outputs
#endif  // VINO_CORE_LIB_OUTPUTS_BASE_OUTPUT_H
