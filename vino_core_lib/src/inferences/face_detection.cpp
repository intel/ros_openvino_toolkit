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
 * @brief a header file with declaration of FaceDetection class and
 * FaceDetectionResult class
 * @file face_detection.cpp
 */

#include <memory>
#include <string>
#include <vector>

#include "vino_core_lib/inferences/face_detection.h"
#include "vino_core_lib/outputs/base_output.h"
#include "vino_core_lib/slog.h"

using namespace vino_core_lib;

// FaceDetectionResult
FaceDetectionResult::FaceDetectionResult(const cv::Rect& location) : Result(location)
{
}

// FaceDetection
FaceDetection::FaceDetection(bool enable_roi_constraint, double show_output_thresh)
  : ObjectDetection(enable_roi_constraint, show_output_thresh)
{
}


// FaceDetectionResultFilter
FaceDetectionResultFilter::FaceDetectionResultFilter()
{
}

void FaceDetectionResultFilter::init()
{
  key_to_function_.insert(std::make_pair("label", isValidLabel));
  key_to_function_.insert(std::make_pair("confidence", isValidConfidence));
}

void FaceDetectionResultFilter::acceptResults(const std::vector<Result>& results)
{
  results_ = results;
}

std::vector<cv::Rect> FaceDetectionResultFilter::getFilteredLocations()
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

bool FaceDetectionResultFilter::isValidLabel(const Result& result, const std::string& op,
                                                                 const std::string& target)
{
  return stringCompare(result.getLabel(), op, target);
}

bool FaceDetectionResultFilter::isValidConfidence(const Result& result, const std::string& op,
                                                                      const std::string& target)
{
  return floatCompare(result.getConfidence(), op, stringToFloat(target));
}

bool FaceDetectionResultFilter::isValidResult(const Result& result)
{
  ISVALIDRESULT(key_to_function_, result);
}

using namespace vino_core_lib;
REG_INFERENCE(FaceDetection, "FaceDetection");
