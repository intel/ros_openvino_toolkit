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
#include "vino_core_lib/inferences/object_detection.h"
#include "vino_core_lib/outputs/base_output.h"
#include "vino_core_lib/slog.h"
// ObjectDetectionResult
vino_core_lib::ObjectDetectionResult::ObjectDetectionResult(
    const cv::Rect& location)
    : Result(location){}
// ObjectDetection
vino_core_lib::ObjectDetection::ObjectDetection(bool enable_roi_constraint, 
                                                  double show_output_thresh)
    : vino_core_lib::BaseInference(),
      show_output_thresh_(show_output_thresh){}
vino_core_lib::ObjectDetection::~ObjectDetection() = default;
void vino_core_lib::ObjectDetection::loadNetwork(
    const std::shared_ptr<Models::ObjectDetectionModel> network) {
  valid_model_ = network;
  max_proposal_count_ = network->getMaxProposalCount();
  object_size_ = network->getObjectSize();
  setMaxBatchSize(network->getMaxBatchSize());
}
bool vino_core_lib::ObjectDetection::enqueue(const cv::Mat& frame,
                                              const cv::Rect& input_frame_loc) {
  if (width_ == 0 && height_ == 0) {
    width_ = frame.cols;
    height_ = frame.rows;
  }
  if (!vino_core_lib::BaseInference::enqueue<u_int8_t>(
          frame, input_frame_loc, 1, 0, valid_model_->getInputName())) {
    return false;
  }
  Result r(input_frame_loc);
  results_.clear();
  results_.emplace_back(r);
  return true;
}
bool vino_core_lib::ObjectDetection::submitRequest() {
  return vino_core_lib::BaseInference::submitRequest();
}
bool vino_core_lib::ObjectDetection::fetchResults() {
  bool can_fetch = vino_core_lib::BaseInference::fetchResults();
  if (!can_fetch) return false;
  bool found_result = false;
  results_.clear();
  InferenceEngine::InferRequest::Ptr request = getEngine()->getRequest();
  std::string output = valid_model_->getOutputName();
  const float* detections = request->GetBlob(output)->buffer().as<float*>();
  for (int i = 0; i < max_proposal_count_; i++) {
    float image_id = detections[i * object_size_ + 0];
    cv::Rect r;
    auto label_num = static_cast<unsigned int>(detections[i * object_size_ + 1]);
    std::vector<std::string>& labels = valid_model_->getLabels();
    r.x = static_cast<int>(detections[i * object_size_ + 3] * width_);
    r.y = static_cast<int>(detections[i * object_size_ + 4] * height_);
    r.width = static_cast<int>(detections[i * object_size_ + 5] * width_ - r.x);
    r.height =
        static_cast<int>(detections[i * object_size_ + 6] * height_ - r.y);
    Result result(r);
    result.label_ = label_num < labels.size()
                        ? labels[label_num]
                        : std::string("label #") + std::to_string(label_num);
    result.confidence_ = detections[i * object_size_ + 2];
    if (result.confidence_ <= show_output_thresh_) {
      continue;
    }
    if (image_id < 0) {
      break;
    }
    found_result = true;
    results_.emplace_back(result);
  }
  if (!found_result) results_.clear();
  return true;
}
const int vino_core_lib::ObjectDetection::getResultsLength() const {
  return static_cast<int>(results_.size());
}
const vino_core_lib::Result*
vino_core_lib::ObjectDetection::getLocationResult(int idx) const {
  return &(results_[idx]);
}
const std::string vino_core_lib::ObjectDetection::getName() const {
  return valid_model_->getModelName();
}
const void vino_core_lib::ObjectDetection::observeOutput(
    const std::shared_ptr<Outputs::BaseOutput>& output) {
  if (output != nullptr) {
    output->accept(results_);
  }
}
