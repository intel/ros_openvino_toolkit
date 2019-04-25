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
 * @brief A header file with declaration for ObjectDetectionModel Class
 * @file face_detection_model.h
 */
#ifndef DYNAMIC_VINO_LIB_MODELS_OBJECT_DETECTION_YOLOV2VOC_MODEL_H
#define DYNAMIC_VINO_LIB_MODELS_OBJECT_DETECTION_YOLOV2VOC_MODEL_H
#include <string>
#include "dynamic_vino_lib/models/base_model.h"
namespace Models {
/**
 * @class ObjectDetectionModel
 * @brief This class generates the face detection model.
 */
class ObjectDetectionYolov2vocModel : public ObjectDetectionModel {
 public:
  ObjectDetectionYolov2vocModel(const std::string&, int, int, int);
  inline const std::string getInputName() { return input_; }
  inline const std::string getOutputName() { return output_; }
  InferenceEngine::CNNLayerPtr getLayer() { return output_layer_; }
  InferenceEngine::InputInfo::Ptr getInputInfo() { return input_info_; }
  /**
   * @brief Get the name of this detection model.
   * @return Name of the model.
   */
  const std::string getModelName() const override;

 protected:
  void checkLayerProperty(const InferenceEngine::CNNNetReader::Ptr&) override;
  void setLayerProperty(InferenceEngine::CNNNetReader::Ptr) override;

  std::string input_;
  std::string output_;

  InferenceEngine::CNNLayerPtr output_layer_;
  InferenceEngine::InputInfo::Ptr input_info_;
};
}  // namespace Models
#endif  // DYNAMIC_VINO_LIB_MODELS_OBJECT_DETECTION_YOLOV2VOC_MODEL_H
