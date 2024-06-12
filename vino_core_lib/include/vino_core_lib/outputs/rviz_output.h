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
* @brief A header file with declaration for RvizOutput Class
* @file image_window_output.h
*/

#ifndef VINO_CORE_LIB__OUTPUTS__RVIZ_OUTPUT_H
#define VINO_CORE_LIB__OUTPUTS__RVIZ_OUTPUT_H

#include <vector>
#include <string>
#include <memory>

#include "vino_core_lib/outputs/base_output.h"
#include "vino_core_lib/outputs/image_window_output.h"

namespace Outputs
{
/**
 * @class RvizOutput
 * @brief This class handles and shows the detection result with rviz.
 */
class RvizOutput : public BaseOutput
{
public:
  RvizOutput() {};
  RvizOutput(const std::string& pipeline_name) : pipeline_name_(pipeline_name) {};

  void init(const std::string& pipeline_name) override;
  /**
   * @brief Construct frame for rviz
   * @param[in] A frame.
   */
  void feedFrame(const cv::Mat&) override;
  /**
   * @brief Show all the contents generated by the accept
   * functions with rviz.
   */
  void handleOutput() override;
  /**
   * @brief Generate image window output content according to
   * the license plate detection result.
   * @param[in] A license plate detection result objetc.
   */
  void accept(const std::vector<vino_core_lib::LicensePlateDetectionResult>&) override;
  /**
   * @brief Generate image window output content according to
   * the vehicle attributes detection result.
   * @param[in] A vehicle attributes detection result objetc.
   */
  void accept(const std::vector<vino_core_lib::VehicleAttribsDetectionResult>&) override;
  /**
   * @brief Generate rviz output content according to
   * the person attributes detection result.
   * @param[in] A person attributes detection result objetc.
   */
  void accept(const std::vector<vino_core_lib::PersonAttribsDetectionResult>&) override;
  /**
   * @brief Generate rviz output content according to
   * the face reidentification result.
   * @param[in] A face reidentification result objetc.
   */
  void accept(const std::vector<vino_core_lib::FaceReidentificationResult>&) override;
  /**
   * @brief Generate rviz output content according to
   * the face detection result.
   * @param[in] A face detection result objetc.
   */
  void accept(const std::vector<vino_core_lib::FaceDetectionResult>&) override;
  /**
   * @brief Generate rviz output content according to
   * the object detection result.
   * @param[in] results A bundle of object detection results.
   */
  void accept(const std::vector<vino_core_lib::ObjectDetectionResult>&) override;
  /**
 * @brief Generate rviz output content according to
 * the object segmentation result.
 * @param[in] results A bundle of object segmentation results.
 */
  void accept(const std::vector<vino_core_lib::EmotionsResult>&) override;
  /**
   * @brief Generate rviz output content according to
   * the age and gender detection result.
   * @param[in] A head pose detection result objetc.
   */
  void accept(const std::vector<vino_core_lib::HeadPoseResult>&) override;
  /**
   * @brief Generate rviz output content according to
   * the headpose detection result.
   * @param[in] An age gender detection result objetc.
   */
  void accept(const std::vector<vino_core_lib::AgeGenderResult>&) override;

  /**
   * @brief Generate  rviz output content according to
   * the object segmentation result.
   * @param[in] An object segmentation result objetc.
   */
  void accept(const std::vector<vino_core_lib::ObjectSegmentationResult>&) override;
  /**
  * @brief Generate  rviz output content according to
  * the person re-ID result.
  * @param[in] An object segmentation result objetc.
  */
  void accept(const std::vector<vino_core_lib::PersonReidentificationResult>&) override;
  /**
    * @brief Merge mask for image window ouput
    * the object segmentation result.
    * @param[in] An object segmentation result objetc.
    */

private:
  std_msgs::Header getHeader();
  std::string pipeline_name_;
  ros::NodeHandle nh_;
  ros::Publisher pub_image_;
  sensor_msgs::Image::Ptr image_topic_;
  std::shared_ptr<Outputs::ImageWindowOutput> image_window_output_;
};
}  // namespace Outputs
#endif  // VINO_CORE_LIB__OUTPUTS__RVIZ_OUTPUT_H
