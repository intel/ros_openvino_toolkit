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
 * @brief a header file with declaration of ObjectDetection class and
 * ObjectDetectionResult class
 * @file object_detection.cpp
 */
#include <memory>
#include <string>
#include <vector>
#include <utility>
#include <stack>
#include "vino_core_lib/inferences/object_detection.h"
#include "vino_core_lib/outputs/base_output.h"
#include "vino_core_lib/slog.h"

using namespace vino_core_lib;

// ObjectDetectionResult
ObjectDetectionResult::ObjectDetectionResult(const cv::Rect& location) : Result(location)
{
}

// ObjectDetection
ObjectDetection::ObjectDetection(bool enable_roi_constraint, double show_output_thresh)
  : show_output_thresh_(show_output_thresh)
  , enable_roi_constraint_(enable_roi_constraint)
  , BaseInference()
{
  result_filter_ = std::make_shared<Filter>();
  result_filter_->init();
}

void ObjectDetection::loadNetwork(std::shared_ptr<Models::BaseModel> network)
{
  valid_model_ = std::dynamic_pointer_cast<Models::ObjectDetectionModel>(network);

  setMaxBatchSize(network->getMaxBatchSize());
}

// void ObjectDetection::loadNetwork(std::shared_ptr<Models::ObjectDetectionModel> network)
// {
//   valid_model_ = network;

//   setMaxBatchSize(network->getMaxBatchSize());
// }

bool ObjectDetection::enqueue(const cv::Mat& frame, const cv::Rect& input_frame_loc)
{
  if (valid_model_ == nullptr || getEngine() == nullptr)
  {
    return false;
  }

  if (enqueued_frames_ >= valid_model_->getMaxBatchSize())
  {
    slog::warn << "Number of " << getName() << "input more than maximum(" << max_batch_size_
               << ") processed by inference" << slog::endl;
    return false;
  }

  if (!valid_model_->enqueue(getEngine(), frame, input_frame_loc))
  {
    return false;
  }

  // nonsense!!
  // Result r(input_frame_loc);
  // results_.clear();
  // results_.emplace_back(r);
  enqueued_frames_ += 1;
  return true;
}

bool ObjectDetection::fetchResults()
{
  bool can_fetch = BaseInference::fetchResults();
  if (!can_fetch)
  {
    return false;
  }

  results_.clear();

  return (valid_model_ != nullptr) &&
         valid_model_->fetchResults(getEngine(), results_, show_output_thresh_, enable_roi_constraint_);
}

int ObjectDetection::getResultsLength() const
{
  return static_cast<int>(results_.size());
}

const ObjectDetection::Result* ObjectDetection::getLocationResult(int idx) const
{
  return &(results_[idx]);
}

const std::string ObjectDetection::getName() const
{
  return valid_model_->getModelCategory();
}

void ObjectDetection::observeOutput(const std::shared_ptr<Outputs::BaseOutput>& output)
{
  if (output != nullptr)
  {
    output->accept(results_);
  }
}

const std::vector<cv::Rect>
ObjectDetection::getFilteredROIs(const std::string filter_conditions) const
{
  if (!result_filter_->isValidFilterConditions(filter_conditions))
  {
    std::vector<cv::Rect> filtered_rois;
    for (auto result : results_)
    {
      filtered_rois.push_back(result.getLocation());
    }
    return filtered_rois;
  }
  result_filter_->acceptResults(results_);
  result_filter_->acceptFilterConditions(filter_conditions);
  return result_filter_->getFilteredLocations();
}

// ObjectDetectionResultFilter
ObjectDetectionResultFilter::ObjectDetectionResultFilter()
{
}

void ObjectDetectionResultFilter::init()
{
  key_to_function_.insert(std::make_pair("label", isValidLabel));
  key_to_function_.insert(std::make_pair("confidence", isValidConfidence));
}

void ObjectDetectionResultFilter::acceptResults(const std::vector<Result>& results)
{
  results_ = results;
}

std::vector<cv::Rect> ObjectDetectionResultFilter::getFilteredLocations()
{
  std::vector<cv::Rect> locations;
  for (auto result : results_)
  {
    if (isValidResult(result))
    {
      locations.push_back(result.getLocation());
    }
  }
  return locations;
}

bool ObjectDetectionResultFilter::isValidLabel(const Result& result, const std::string& op,
                                                                 const std::string& target)
{
  return stringCompare(result.getLabel(), op, target);
}

bool ObjectDetectionResultFilter::isValidConfidence(const Result& result, const std::string& op,
                                                                      const std::string& target)
{
  return floatCompare(result.getConfidence(), op, stringToFloat(target));
}

bool ObjectDetectionResultFilter::isValidResult(const Result& result)
{
  ISVALIDRESULT(key_to_function_, result);
}

double ObjectDetection::calcIoU(const cv::Rect& box_1, const cv::Rect& box_2)
{
  cv::Rect i = box_1 & box_2;
  cv::Rect u = box_1 | box_2;

  return static_cast<double>(i.area()) / static_cast<double>(u.area());
}

using namespace vino_core_lib;
REG_INFERENCE(ObjectDetection, "ObjectDetection");
