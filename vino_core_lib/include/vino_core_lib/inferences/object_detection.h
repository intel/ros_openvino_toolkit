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
#ifndef VINO_CORE_LIB__INFERENCES__OBJECT_DETECTION_H
#define VINO_CORE_LIB__INFERENCES__OBJECT_DETECTION_H
#include <object_msgs/Object.h>
#include <object_msgs/ObjectInBox.h>
#include <object_msgs/ObjectsInBoxes.h>
#include <memory>
#include <vector>
#include <string>
#include <map>
#include "vino_core_lib/engines/engine.h"
#include "vino_core_lib/inferences/base_inference.h"
#include "vino_core_lib/inferences/base_filter.h"
#include "inference_engine.hpp"
#include "opencv2/opencv.hpp"
// namespace
namespace vino_core_lib
{
/**
 * @class ObjectDetectionResult
 * @brief Class for storing and processing face detection result.
 */
class ObjectDetectionResult : public Result
{
public:
  friend class ObjectDetection;
  explicit ObjectDetectionResult(const cv::Rect& location);
  std::string getLabel() const
  {
    return label_;
  }

  void setLabel(const std::string& label)
  {
    label_ = label;
  }
  /**
   * @brief Get the confidence that the detected area is a face.
   * @return The confidence value.
   */
  float getConfidence() const
  {
    return confidence_;
  }

  void setConfidence(const float& con)
  {
    confidence_ = con;
  }

  bool operator<(const ObjectDetectionResult& s2) const
  {
    return this->confidence_ > s2.confidence_;
  }

private:
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
  using Result = vino_core_lib::ObjectDetectionResult;

  ObjectDetectionResultFilter();

  /**
   * @brief Initiate the object detection result filter.
   */
  void init() override;
  /**
   * @brief Set the object detection results into filter.
   * @param[in] The object detection results.
   */
  void acceptResults(const std::vector<Result>& results);
  /**
   * @brief Get the filtered results' ROIs.
   * @return The filtered ROIs.
   */
  std::vector<cv::Rect> getFilteredLocations() override;

private:
  /**
   * @brief Decide whether a result is valid for label filter condition.
   * @param[in] Result to be decided, filter operator, target label value.
   * @return True if valid, false if not.
   */
  static bool isValidLabel(const Result& result, const std::string& op, const std::string& target);
  /**
   * @brief Decide whether a result is valid for confidence filter condition.
   * @param[in] Result to be decided, filter operator, target confidence value.
   * @return True if valid, false if not.
   */
  static bool isValidConfidence(const Result& result, const std::string& op, const std::string& target);

  /**
   * @brief Decide whether a result is valid.
   * @param[in] Result to be decided.
   * @return True if valid, false if not.
   */
  bool isValidResult(const Result& result);

  std::map<std::string, bool (*)(const Result&, const std::string&, const std::string&)> key_to_function_;
  std::vector<Result> results_;
};

/**
 * @class ObjectDetection
 * @brief Class to load face detection model and perform face detection.
 */
class ObjectDetection : public BaseInference
{
public:
  using Result = vino_core_lib::ObjectDetectionResult;
  using Filter = vino_core_lib::ObjectDetectionResultFilter;
  explicit ObjectDetection(bool, double);
  ObjectDetection(){};
  ~ObjectDetection() override {};

  /**
   * @brief Load the face detection model.
   */
  void loadNetwork(std::shared_ptr<Models::BaseModel>) override;

  /**
   * @brief Load the face detection model.
   */
  // void loadNetwork(std::shared_ptr<Models::ObjectDetectionModel>);
  /**
   * @brief Enqueue a frame to this class.
   * The frame will be buffered but not infered yet.
   * @param[in] frame The frame to be enqueued.
   * @param[in] input_frame_loc The location of the enqueued frame with respect
   * to the frame generated by the input device.
   * @return Whether this operation is successful.
   */
  bool enqueue(const cv::Mat&, const cv::Rect&) override;

  /**
   * @brief This function will fetch the results of the previous inference and
   * stores the results in a result buffer array. All buffered frames will be
   * cleared.
   * @return Whether the Inference object fetches a result this time
   */
  bool fetchResults() override;
  /**
   * @brief Get the length of the buffer result array.
   * @return The length of the buffer result array.
   */
  int getResultsLength() const override;
  /**
   * @brief Get the location of result with respect
   * to the frame generated by the input device.
   * @param[in] idx The index of the result.
   */
  const Result* getLocationResult(int idx) const override;
  /**
   * @brief Show the observed detection result either through image window
     or ROS topic.
   */
  void observeOutput(const std::shared_ptr<Outputs::BaseOutput>& output);
  /**
   * @brief Get the name of the Inference instance.
   * @return The name of the Inference instance.
   */
  const std::string getName() const override;

  const std::vector<cv::Rect> getFilteredROIs(const std::string filter_conditions) const override;
  /**
   * @brief Calculate the IoU ratio for the given rectangles.
   * @return IoU Ratio of the given rectangles.
   */
  static double calcIoU(const cv::Rect& box_1, const cv::Rect& box_2);

private:
  std::shared_ptr<Models::ObjectDetectionModel> valid_model_;
  std::shared_ptr<Filter> result_filter_;
  std::vector<Result> results_;
  int width_ = 0;
  int height_ = 0;
  int max_proposal_count_;
  int object_size_;
  double show_output_thresh_ = 0;
  bool enable_roi_constraint_ = false;
};
}  // namespace vino_core_lib
#endif  // VINO_CORE_LIB__INFERENCES__OBJECT_DETECTION_H
