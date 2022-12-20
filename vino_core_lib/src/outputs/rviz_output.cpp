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
 * @brief a header file with declaration of ImageWindowOutput class
 * @file image_window_output.cpp
 */

#include <algorithm>
#include <string>
#include <vector>
#include <memory>
#include "cv_bridge/cv_bridge.h"
#include "vino_core_lib/pipeline.h"
#include "vino_core_lib/outputs/rviz_output.h"

void Outputs::RvizOutput::init(const std::string& pipeline_name)
{
  pipeline_name_ = pipeline_name;
  image_topic_ = nullptr;
  pub_image_ = nh_.advertise<sensor_msgs::Image>("/openvino_toolkit/" + pipeline_name + "/images", 16);
  image_window_output_ = std::make_shared<Outputs::ImageWindowOutput>(pipeline_name, 950);
}

void Outputs::RvizOutput::feedFrame(const cv::Mat& frame)
{
  image_window_output_->feedFrame(frame);
}

void Outputs::RvizOutput::accept(const std::vector<vino_core_lib::LicensePlateDetectionResult>& results)
{
  image_window_output_->accept(results);
}
void Outputs::RvizOutput::accept(const std::vector<vino_core_lib::VehicleAttribsDetectionResult>& results)
{
  image_window_output_->accept(results);
}

void Outputs::RvizOutput::accept(const std::vector<vino_core_lib::PersonAttribsDetectionResult>& results)
{
  image_window_output_->accept(results);
}

void Outputs::RvizOutput::accept(const std::vector<vino_core_lib::FaceReidentificationResult>& results)
{
  image_window_output_->accept(results);
}

void Outputs::RvizOutput::accept(const std::vector<vino_core_lib::FaceDetectionResult>& results)
{
  image_window_output_->accept(results);
}

void Outputs::RvizOutput::accept(const std::vector<vino_core_lib::ObjectDetectionResult>& results)
{
  image_window_output_->accept(results);
}

void Outputs::RvizOutput::accept(const std::vector<vino_core_lib::EmotionsResult>& results)
{
  image_window_output_->accept(results);
}

void Outputs::RvizOutput::accept(const std::vector<vino_core_lib::AgeGenderResult>& results)
{
  image_window_output_->accept(results);
}

void Outputs::RvizOutput::accept(const std::vector<vino_core_lib::HeadPoseResult>& results)
{
  image_window_output_->accept(results);
}

void Outputs::RvizOutput::accept(const std::vector<vino_core_lib::ObjectSegmentationResult>& results)
{
  image_window_output_->accept(results);
}
void Outputs::RvizOutput::accept(const std::vector<vino_core_lib::PersonReidentificationResult>& results)
{
  image_window_output_->accept(results);
}

void Outputs::RvizOutput::handleOutput()
{
  image_window_output_->setPipeline(getPipeline());
  image_window_output_->decorateFrame();
  cv::Mat frame = image_window_output_->getFrame();
  std_msgs::Header header = getHeader();
  // std_msgs::Header header = getPipeline()->getInputDevice()->getLockedHeader();
  std::shared_ptr<cv_bridge::CvImage> cv_ptr = std::make_shared<cv_bridge::CvImage>(header, "bgr8", frame);
  sensor_msgs::Image image_msg;
  image_topic_ = cv_ptr->toImageMsg();
  pub_image_.publish(image_topic_);
}

#if 1   // deprecated
/**
 * Don't use this inferface to create new time stamp, it'd better use camera/topic
 * time stamp.
 */
std_msgs::Header Outputs::RvizOutput::getHeader()
{
  std_msgs::Header header;
  //deprecated!!
  header.frame_id = "rviz_output";

  std::chrono::high_resolution_clock::time_point tp =
      std::chrono::high_resolution_clock::now();
  int64 ns = tp.time_since_epoch().count();
  header.stamp.sec = ns / 1000000000;
  header.stamp.nsec = ns % 1000000000;
  return header;
}
#endif  // depreated 

REG_OUTPUT(RvizOutput, "RViz");