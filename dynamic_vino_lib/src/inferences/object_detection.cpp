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
#include "dynamic_vino_lib/inferences/object_detection.h"
#include "dynamic_vino_lib/outputs/base_output.h"
#include "dynamic_vino_lib/slog.h"
// ObjectDetectionResult
dynamic_vino_lib::ObjectDetectionResult::ObjectDetectionResult(
    const cv::Rect& location) : Result(location){}

// ObjectDetectionResultFilter
dynamic_vino_lib::ObjectDetectionResultFilter::ObjectDetectionResultFilter() {}

void dynamic_vino_lib::ObjectDetectionResultFilter::init()
{
  key_to_function_.insert(std::make_pair("label", isValidLabel));
  key_to_function_.insert(std::make_pair("confidence", isValidConfidence));
}

void dynamic_vino_lib::ObjectDetectionResultFilter::acceptResults(
  const std::vector<Result> & results)
{
  results_ = results;
}

std::vector<dynamic_vino_lib::ObjectDetectionResult>
dynamic_vino_lib::ObjectDetectionResultFilter::getFilteredResults()
{
  std::vector<Result> results;
  for (auto result : results_) {
    if (isValidResult(result)) {
      results.push_back(result);
    }
  }
  return results;
}

// std::vector<cv::Rect>
// dynamic_vino_lib::ObjectDetectionResultFilter::getFilteredLocations()
// {
//   std::vector<cv::Rect> locations;
//   for (auto result : results_) {
//     if (isValidResult(result)) {
//       locations.push_back(result.getLocation());
//     }
//   }
//   return locations;
// }

bool dynamic_vino_lib::ObjectDetectionResultFilter::isValidLabel(
  const Result & result, const std::string & op, const std::string & target)
{
  return stringCompare(result.getLabel(), op, target);
}

bool dynamic_vino_lib::ObjectDetectionResultFilter::isValidConfidence(
  const Result & result, const std::string & op, const std::string & target)
{
  return floatCompare(result.getConfidence(), op, stringToFloat(target));
}

bool dynamic_vino_lib::ObjectDetectionResultFilter::isValidResult(
  const Result & result)
{
  ISVALIDRESULT(key_to_function_, result);
}