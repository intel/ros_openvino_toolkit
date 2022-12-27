// Copyright (c) 2018 Intel Corporation
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
/**
 * @brief A header file with declaration for ObjectSegmentationModel Class
 * @file face_detection_model.h
 */
#ifndef VINO_CORE_LIB__MODELS__OBJECT_SEGMENTATION_MODEL_H
#define VINO_CORE_LIB__MODELS__OBJECT_SEGMENTATION_MODEL_H
#include <string>
#include "vino_core_lib/models/base_model.h"
namespace Models
{
/**
 * @class ObjectSegmentationModel
 * @brief This class generates the object segmentation model.
 */
class ObjectSegmentationModel : public BaseModel
{
public:
  ObjectSegmentationModel(const std::string& label_loc, const std::string& model_loc, int batch_size = 1);
  inline int getMaxProposalCount() const
  {
    return max_proposal_count_;
  }
  inline int getObjectSize() const
  {
    return object_size_;
  }

  bool enqueue(const std::shared_ptr<Engines::Engine>&, const cv::Mat&, const cv::Rect&) override;

  bool matToBlob(const cv::Mat&, const cv::Rect&, float, int, const std::shared_ptr<Engines::Engine>&);

  /**
   * @brief Get the name of this segmentation model.
   * @return Name of the model.
   */
  const std::string getModelCategory() const override;
  bool updateLayerProperty(std::shared_ptr<ov::Model>&) override;

private:
  int max_proposal_count_;
  int object_size_;
  ov::Shape input_shape_;
};
}  // namespace Models
#endif  // VINO_CORE_LIB__MODELS__OBJECT_SEGMENTATION_MODEL_H
