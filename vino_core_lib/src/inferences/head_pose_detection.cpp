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
 * @brief a header file with declaration of HeadPoseDetection class and
 * HeadPoseResult class
 * @file head_pose_recognition.cpp
 */

#include "vino_core_lib/inferences/head_pose_detection.h"
#include <memory>
#include <string>
#include "vino_core_lib/outputs/base_output.h"

using namespace vino_core_lib;

// HeadPoseResult
HeadPoseResult::HeadPoseResult(const cv::Rect& location) : Result(location)
{
}

void HeadPoseDetection::loadNetwork(std::shared_ptr<Models::BaseModel> network)
{
  valid_model_ = std::dynamic_pointer_cast<Models::HeadPoseDetectionModel>(network);

  setMaxBatchSize(network->getMaxBatchSize());
}

// // Head Pose Detection
// void HeadPoseDetection::loadNetwork(std::shared_ptr<Models::HeadPoseDetectionModel> network)
// {
//   valid_model_ = network;
//   setMaxBatchSize(network->getMaxBatchSize());
// }

bool HeadPoseDetection::enqueue(const cv::Mat& frame, const cv::Rect& input_frame_loc)
{
  if (getEnqueuedNum() == 0)
  {
    results_.clear();
  }
  bool succeed = BaseInference::enqueue<uint8_t>(frame, input_frame_loc, 1, getResultsLength(),
                                                                   valid_model_->getInputName());
  if (!succeed)
    return false;
  Result r(input_frame_loc);
  results_.emplace_back(r);
  return true;
}

bool HeadPoseDetection::submitRequest()
{
  return BaseInference::submitRequest();
}

bool HeadPoseDetection::fetchResults()
{
  bool can_fetch = BaseInference::fetchResults();
  if (!can_fetch)
    return false;
  auto request = getEngine()->getRequest();
  InferenceEngine::Blob::Ptr angle_r = request->GetBlob(valid_model_->getOutputOutputAngleR());
  InferenceEngine::Blob::Ptr angle_p = request->GetBlob(valid_model_->getOutputOutputAngleP());
  InferenceEngine::Blob::Ptr angle_y = request->GetBlob(valid_model_->getOutputOutputAngleY());

  for (int i = 0; i < getResultsLength(); ++i)
  {
    results_[i].angle_r_ = angle_r->buffer().as<float*>()[i];
    results_[i].angle_p_ = angle_p->buffer().as<float*>()[i];
    results_[i].angle_y_ = angle_y->buffer().as<float*>()[i];
  }
  return true;
}

int HeadPoseDetection::getResultsLength() const
{
  return static_cast<int>(results_.size());
}

const Result* HeadPoseDetection::getLocationResult(int idx) const
{
  return &(results_[idx]);
}

const std::string HeadPoseDetection::getName() const
{
  return valid_model_->getModelCategory();
}

void HeadPoseDetection::observeOutput(const std::shared_ptr<Outputs::BaseOutput>& output)
{
  if (output != nullptr)
  {
    output->accept(results_);
  }
}

const std::vector<cv::Rect>
HeadPoseDetection::getFilteredROIs(const std::string filter_conditions) const
{
  if (!filter_conditions.empty())
  {
    slog::err << "Headpose detection does not support filtering now! "
              << "Filter conditions: " << filter_conditions << slog::endl;
  }
  std::vector<cv::Rect> filtered_rois;
  for (auto res : results_)
  {
    filtered_rois.push_back(res.getLocation());
  }
  return filtered_rois;
}

using namespace vino_core_lib;
REG_INFERENCE(HeadPoseDetection, "HeadPoseEstimation");
