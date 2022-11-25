
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
 * @brief a header file with declaration of FaceDetectionModel class
 * @file face_detection_model.cpp
 */

#include <string>

#include "vino_core_lib/models/face_detection_model.h"
#include "vino_core_lib/slog.h"

// Validated Face Detection Network
Models::FaceDetectionModel::FaceDetectionModel(const std::string& label_loc, const std::string& model_loc, int max_batch_size)
  : ObjectDetectionModel(label_loc, model_loc, max_batch_size)
{
}

const std::string Models::FaceDetectionModel::getModelCategory() const
{
  return "Face Detection";
}

bool Models::FaceDetectionModel::enqueue(const std::shared_ptr<Engines::Engine>& engine, const cv::Mat& frame,
                                              const cv::Rect& input_frame_loc)
{
  if (!this->matToBlob(frame, input_frame_loc, 1, 0, engine))
  {
    return false;
  }

  setFrameSize(frame.cols, frame.rows);
  return true;
}

bool Models::FaceDetectionModel::matToBlob(const cv::Mat& orig_image, const cv::Rect&, float scale_factor,
                                                int batch_index, const std::shared_ptr<Engines::Engine>& engine)
{
  return true;
}

bool Models::FaceDetectionModel::fetchResults(const std::shared_ptr<Engines::Engine>& engine,
                                                   std::vector<vino_core_lib::ObjectDetectionResult>& results,
                                                   const float& confidence_thresh, const bool& enable_roi_constraint)
{
  return true;
}

bool Models::FaceDetectionModel::fetchResults(const std::shared_ptr<Engines::Engine>& engine,
                                                   std::vector<vino_core_lib::FaceDetectionResult>& results,
                                                   const float& confidence_thresh, const bool& enable_roi_constraint)
{
  return true;
}

bool Models::FaceDetectionModel::updateLayerProperty(InferenceEngine::CNNNetwork& net_reader)
{
  return true;
}

using namespace Models;

REG_MODEL(FaceDetectionModel, "FaceDetection");
