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
 * @brief a header file with declaration of AgeGenderResult class
 * @file age_gender_detection.cpp
 */

#include <memory>
#include <string>
#include <vector>

#include "vino_core_lib/inferences/age_gender_detection.h"
#include "vino_core_lib/outputs/base_output.h"

using namespace vino_core_lib;

// AgeGenderResult
AgeGenderResult::AgeGenderResult(const cv::Rect& location) : Result(location) {}

void AgeGenderDetection::loadNetwork(std::shared_ptr<Models::BaseModel> network)
{
  valid_model_ = std::dynamic_pointer_cast<Models::AgeGenderDetectionModel>(network);

  setMaxBatchSize(network->getMaxBatchSize());
}

// // AgeGender Detection
// void AgeGenderDetection::loadNetwork(std::shared_ptr<Models::AgeGenderDetectionModel> network)
// {
//   valid_model_ = network;
//   setMaxBatchSize(network->getMaxBatchSize());
// }

bool AgeGenderDetection::enqueue(const cv::Mat& frame, const cv::Rect& input_frame_loc)
{
  if (getEnqueuedNum() == 0)
  {
    results_.clear();
  }
  bool succeed = BaseInference::enqueue<float>(frame, input_frame_loc, 1, getResultsLength(),
                                                                 valid_model_->getInputName());
  if (!succeed) return false;

  Result r(input_frame_loc);
  results_.emplace_back(r);
  return true;
}

bool AgeGenderDetection::submitRequest()
{
  return BaseInference::submitRequest();
}

bool AgeGenderDetection::fetchResults()
{
  bool can_fetch = BaseInference::fetchResults();

  if (!can_fetch) return false;

  auto request = getEngine()->getRequest();
  InferenceEngine::Blob::Ptr genderBlob = request->GetBlob(valid_model_->getOutputGenderName());
  InferenceEngine::Blob::Ptr ageBlob = request->GetBlob(valid_model_->getOutputAgeName());

  for (size_t i = 0; i < results_.size(); ++i)
  {
    results_[i].age_ = ageBlob->buffer().as<float*>()[i] * 100;
    results_[i].male_prob_ = genderBlob->buffer().as<float*>()[i * 2 + 1];
  }
  
  return true;
}

int AgeGenderDetection::getResultsLength() const
{
  return static_cast<int>(results_.size());
}

const Result* AgeGenderDetection::getLocationResult(int idx) const
{
  return &(results_[idx]);
}

const std::string AgeGenderDetection::getName() const
{
  return valid_model_->getModelCategory();
}

void AgeGenderDetection::observeOutput(const std::shared_ptr<Outputs::BaseOutput>& output)
{
  if (output != nullptr)
  {
    output->accept(results_);
  }
}

const std::vector<cv::Rect>
AgeGenderDetection::getFilteredROIs(const std::string filter_conditions) const
{
  if (!filter_conditions.empty())
  {
    slog::err << "Age gender detection does not support filtering now! "
              << "Filter conditions: " << filter_conditions << slog::endl;
  }
  std::vector<cv::Rect> filtered_rois;
  for (auto res : results_)
  {
    filtered_rois.push_back(res.getLocation());
  }
  return filtered_rois;
}

REG_INFERENCE(AgeGenderDetection, "AgeGenderRecognition");
