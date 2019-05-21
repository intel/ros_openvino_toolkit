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
#include "dynamic_vino_lib/inferences/object_detection_ssd.h"
#include "dynamic_vino_lib/outputs/base_output.h"
#include "dynamic_vino_lib/slog.h"
// ObjectDetectionResult
/*
dynamic_vino_lib::ObjectDetectionResult::ObjectDetectionResult(
    const cv::Rect& location)
    : Result(location){}
*/

dynamic_vino_lib::ObjectDetectionSSD::ObjectDetectionSSD(double show_output_thresh)
  : dynamic_vino_lib::ObjectDetection(),
  show_output_thresh_(show_output_thresh)
{
  result_filter_ = std::make_shared<Filter>();
  result_filter_->init();
}

dynamic_vino_lib::ObjectDetectionSSD::~ObjectDetectionSSD() = default;

void dynamic_vino_lib::ObjectDetectionSSD::loadNetwork(
        std::shared_ptr<Models::ObjectDetectionModel> network) {
  valid_model_ = std::dynamic_pointer_cast<Models::ObjectDetectionSSDModel>(network);

  max_proposal_count_ = network->getMaxProposalCount();
  object_size_ = network->getObjectSize();
  setMaxBatchSize(network->getMaxBatchSize());
}

bool dynamic_vino_lib::ObjectDetectionSSD::enqueue(const cv::Mat& frame,
                                              const cv::Rect& input_frame_loc) {
  if (width_ == 0 && height_ == 0) {
    width_ = frame.cols;
    height_ = frame.rows;
  }

  if (!matToBlob(frame, input_frame_loc, 1, 0, valid_model_->getInputName()))
  {
    return false;
  }

  Result r(input_frame_loc);
  results_.clear();
  results_.emplace_back(r);
  return true;
}

bool dynamic_vino_lib::ObjectDetectionSSD::matToBlob(
    const cv::Mat& orig_image, const cv::Rect&, float scale_factor, 
    int batch_index, const std::string& input_name)
{
  if (enqueued_frames_ == max_batch_size_)
  {
    slog::warn << "Number of " << getName() << "input more than maximum("
               << max_batch_size_ << ") processed by inference" << slog::endl;
    return false;
  }
 
  InferenceEngine::Blob::Ptr input_blob =
      engine_->getRequest()->GetBlob(input_name);

  InferenceEngine::SizeVector blob_size = input_blob->getTensorDesc().getDims();
  const int width = blob_size[3];
  const int height = blob_size[2];
  const int channels = blob_size[1];
  u_int8_t * blob_data = input_blob->buffer().as<u_int8_t*>();

  cv::Mat resized_image(orig_image);
  if (width != orig_image.size().width || height != orig_image.size().height)
  { 
    cv::resize(orig_image, resized_image, cv::Size(width, height));
  }
  int batchOffset = batch_index * width * height * channels;

  for (int c = 0; c < channels; c++)
  { 
    for (int h = 0; h < height; h++)
    {
      for (int w = 0; w < width; w++)
      { 
        blob_data[batchOffset + c * width * height + h * width + w] =
            resized_image.at<cv::Vec3b>(h, w)[c] * scale_factor;
      }
    }
  }

  enqueued_frames_ += 1;
  return true;
}

bool dynamic_vino_lib::ObjectDetectionSSD::submitRequest() {
  return dynamic_vino_lib::BaseInference::submitRequest();
}

bool dynamic_vino_lib::ObjectDetectionSSD::fetchResults() {
  bool can_fetch = dynamic_vino_lib::BaseInference::fetchResults();

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

const int dynamic_vino_lib::ObjectDetectionSSD::getResultsLength() const {
  return static_cast<int>(results_.size());
}

const dynamic_vino_lib::Result*
dynamic_vino_lib::ObjectDetectionSSD::getLocationResult(int idx) const {
  return &(results_[idx]);
}

const std::string dynamic_vino_lib::ObjectDetectionSSD::getName() const {
  return valid_model_->getModelName();
}

const void dynamic_vino_lib::ObjectDetectionSSD::observeOutput(
    const std::shared_ptr<Outputs::BaseOutput>& output,
    const std::string filter_conditions) {
  if (output != nullptr) {
    result_filter_->acceptResults(results_);
    result_filter_->acceptFilterConditions(filter_conditions);
    output->accept(result_filter_->getFilteredResults());
  }
}

const std::vector<cv::Rect> dynamic_vino_lib::ObjectDetectionSSD::getFilteredROIs(
  const std::string filter_conditions) const
{
  result_filter_->acceptResults(results_);
  result_filter_->acceptFilterConditions(filter_conditions);
  std::vector<Result> results = result_filter_->getFilteredResults();
  std::vector<cv::Rect> locations;
  for (auto result : results) {
    locations.push_back(result.getLocation());
  }
  return locations;
}