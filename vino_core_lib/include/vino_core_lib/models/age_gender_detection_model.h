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
 * @brief A header file with declaration for AgeGenderDetectionModel Class
 * @file age_gender_detection_model.h
 */

#ifndef VINO_CORE_LIB__MODELS__AGE_GENDER_DETECTION_MODEL_H
#define VINO_CORE_LIB__MODELS__AGE_GENDER_DETECTION_MODEL_H

#include <string>
#include "vino_core_lib/models/base_model.h"
#include "vino_core_lib/models/model_factory.h"

namespace Models
{
/**
 * @class AgeGenderDetectionModel
 * @brief This class generates the age gender detection model.
 */
class AgeGenderDetectionModel : public BaseModel
{
public:
  AgeGenderDetectionModel() {};

  AgeGenderDetectionModel(const std::string& label_loc, const std::string& model_loc, int batch_size = 1);
  /**
   * @brief Get the input name.
   * @return Input name.
   */

  /**
   * @brief Get the age from the detection reuslt.
   * @return Detected age.
   */
  inline const std::string getOutputAgeName() const
  {
    return getOutputName("age");
  }
  /**
   * @brief Get the gender from the detection reuslt.
   * @return Detected gender.
   */
  inline const std::string getOutputGenderName() const
  {
    return getOutputName("gender");
  }
  /**
   * @brief Get the name of this detection model.
   * @return Name of the model.
   */
  const std::string getModelCategory() const override;

protected:
  bool updateLayerProperty(InferenceEngine::CNNNetwork&) override;
};
}  // namespace Models

#endif  // VINO_CORE_LIB__MODELS__AGE_GENDER_DETECTION_MODEL_H
