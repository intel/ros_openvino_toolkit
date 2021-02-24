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
#include "dynamic_vino_lib/models/base_model.h"
#include <string>
namespace Models
{
/**
 * @class ObjectDetectionModel
 * @brief This class generates the face detection model.
 */
class ObjectDetectionYolov2Model : public ObjectDetectionModel
{
  using Result = dynamic_vino_lib::ObjectDetectionResult;

public:
  ObjectDetectionYolov2Model(const std::string& model_loc, int batch_size = 1);

  bool fetchResults(const std::shared_ptr<Engines::Engine>& engine,
                    std::vector<dynamic_vino_lib::ObjectDetectionResult>& results, const float& confidence_thresh = 0.3,
                    const bool& enable_roi_constraint = false) override;

  bool enqueue(const std::shared_ptr<Engines::Engine>& engine, const cv::Mat& frame,
               const cv::Rect& input_frame_loc) override;

  bool matToBlob(const cv::Mat& orig_image, const cv::Rect&, float scale_factor, int batch_index,
                 const std::shared_ptr<Engines::Engine>& engine) override;

  /**
   * @brief Get the name of this detection model.
   * @return Name of the model.
   */
  const std::string getModelCategory() const override;
  bool updateLayerProperty(InferenceEngine::CNNNetReader::Ptr) override;

protected:
  int getEntryIndex(int side, int lcoords, int lclasses, int location, int entry);
  InferenceEngine::InputInfo::Ptr input_info_ = nullptr;
};
}  // namespace Models
#endif  // DYNAMIC_VINO_LIB_MODELS_OBJECT_DETECTION_YOLOV2VOC_MODEL_H
