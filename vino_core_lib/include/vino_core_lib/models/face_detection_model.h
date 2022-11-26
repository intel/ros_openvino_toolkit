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
 * @brief A header file with declaration for FaceDetectionModel Class
 * @file face_detection_model.h
 */

#ifndef VINO_CORE_LIB__MODELS__FACE_DETECTION_MODEL_H
#define VINO_CORE_LIB__MODELS__FACE_DETECTION_MODEL_H

#include <string>
#include "vino_core_lib/models/base_model.h"
#include "vino_core_lib/inferences/face_detection.h"
#include "vino_core_lib/engines/engine_manager.h"

namespace Models
{
/**
 * @class FaceDetectionModel
 * @brief This class generates the face detection model.
 */
class FaceDetectionModel : public ObjectDetectionModel
{
// class ObjectDetectionResult;
// class FaceDetectionResult;

public:
  FaceDetectionModel() {};
  
  FaceDetectionModel(const std::string& label_loc, const std::string& model_loc, int batch_size = 1);
  
  bool fetchResults(const std::shared_ptr<Engines::Engine>& engine,
                    std::vector<vino_core_lib::ObjectDetectionResult>& results, const float& confidence_thresh = 0.3,
                    const bool& enable_roi_constraint = false) override;
  
  bool fetchResults(const std::shared_ptr<Engines::Engine>& engine,
                    std::vector<vino_core_lib::FaceDetectionResult>& results, const float& confidence_thresh = 0.3,
                    const bool& enable_roi_constraint = false);

  bool enqueue(const std::shared_ptr<Engines::Engine>& engine, const cv::Mat& frame,
               const cv::Rect& input_frame_loc) override;

  bool matToBlob(const cv::Mat& orig_image, const cv::Rect&, float scale_factor, int batch_index,
                 const std::shared_ptr<Engines::Engine>& engine) override;

  /**
   * @brief Get the name of this detection model.
   * @return Name of the model.
   */
  const std::string getModelCategory() const override;

  bool updateLayerProperty(InferenceEngine::CNNNetwork&) override;
};
}  // namespace Models

#endif  // VINO_CORE_LIB__MODELS__FACE_DETECTION_MODEL_H