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
 * @brief A header file with declaration for EmotionDetectionModel Class
 * @file emotion_detection_model.h
 */

#ifndef VINO_CORE_LIB__MODELS__EMOTION_DETECTION_MODEL_H
#define VINO_CORE_LIB__MODELS__EMOTION_DETECTION_MODEL_H

#include <string>
#include "vino_core_lib/models/base_model.h"
#include "vino_core_lib/models/model_factory.h"

namespace Models
{
/**
 * @class EmotionDetectionModel
 * @brief This class generates the emotion detection model.
 */
class EmotionDetectionModel : public BaseModel
{
public:
  EmotionDetectionModel() {};

  EmotionDetectionModel(const std::string& label_loc, const std::string& model_loc, int batch_size = 1);

  /**
   * @brief Get the name of this detection model.
   * @return Name of the model.
   */
  const std::string getModelCategory() const override;
  bool updateLayerProperty(InferenceEngine::CNNNetwork&) override;

private:
  bool verifyOutputLayer(const InferenceEngine::DataPtr& ptr);
};
}  // namespace Models

#endif  // VINO_CORE_LIB__MODELS__EMOTION_DETECTION_MODEL_H
