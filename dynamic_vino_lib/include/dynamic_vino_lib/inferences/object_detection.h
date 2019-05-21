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
 * @brief A header file with declaration for ObjectDetection Class
 * @file object_detection.hpp
 */
#ifndef DYNAMIC_VINO_LIB_INFERENCES_OBJECT_DETECTION_H
#define DYNAMIC_VINO_LIB_INFERENCES_OBJECT_DETECTION_H
#include <object_msgs/Object.h>
#include <object_msgs/ObjectInBox.h>
#include <object_msgs/ObjectsInBoxes.h>
#include <memory>
#include <vector>
#include <string>
#include <map>
#include "dynamic_vino_lib/engines/engine.h"
#include "dynamic_vino_lib/inferences/base_inference.h"
#include "dynamic_vino_lib/inferences/base_filter.h"
#include "inference_engine.hpp"
#include "opencv2/opencv.hpp"
// namespace
namespace dynamic_vino_lib {

class ObjectDetectionResult : public Result {
 public:
  ObjectDetectionResult();
  explicit ObjectDetectionResult(const cv::Rect& location);
  std::string getLabel() const { return label_; }
  /**
   * @brief Get the confidence that the detected area is a face.
   * @return The confidence value. 
   */
  float getConfidence() const { return confidence_; }
  bool operator<(const ObjectDetectionResult &s2) const
  {
    return this->confidence_ > s2.confidence_;
  }

  std::string label_ = "";
  float confidence_ = -1;
};

/**
 * @class ObjectDetectionResultFilter
 * @brief Class for object detection result filter.
 */
class ObjectDetectionResultFilter : public BaseFilter
{
public:
  using Result = dynamic_vino_lib::ObjectDetectionResult;

  ObjectDetectionResultFilter();

  /**
   * @brief Initiate the object detection result filter.
   */
  void init() override;
  /**
   * @brief Set the object detection results into filter.
   * @param[in] The object detection results.
   */
  void acceptResults(const std::vector<Result> & results);
  /**
   * @brief Get the filtered results' ROIs.
   * @return The filtered ROIs.
   */
  std::vector<Result> getFilteredResults();
private:
  /**
   * @brief Decide whether a result is valid for label filter condition.
   * @param[in] Result to be decided, filter operator, target label value.
   * @return True if valid, false if not.
   */
  static bool isValidLabel(
    const Result & result,
    const std::string & op, const std::string & target);
  /**
   * @brief Decide whether a result is valid for confidence filter condition.
   * @param[in] Result to be decided, filter operator, target confidence value.
   * @return True if valid, false if not.
   */
  static bool isValidConfidence(
    const Result & result,
    const std::string & op, const std::string & target);

  /**
   * @brief Decide whether a result is valid.
   * @param[in] Result to be decided.
   * @return True if valid, false if not.
   */
  bool isValidResult(const Result & result);

  std::map<std::string, bool(*)
    (const Result &, const std::string &, const std::string &)> key_to_function_;
  std::vector<Result> results_;
};


class ObjectDetection : public BaseInference
{
 public:
  ObjectDetection() {};
  virtual void loadNetwork(std::shared_ptr<Models::ObjectDetectionModel>) = 0;
};
}  // namespace dynamic_vino_lib
#endif  // DYNAMIC_VINO_LIB_INFERENCES_OBJECT_DETECTION_H
