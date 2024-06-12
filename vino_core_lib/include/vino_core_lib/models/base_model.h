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
 * @brief A header file with declaration for BaseModel Class
 * @file base_model.h
 */

#ifndef VINO_CORE_LIB__MODELS__BASE_MODEL_H
#define VINO_CORE_LIB__MODELS__BASE_MODEL_H

#include <opencv2/opencv.hpp>

#include <memory>
#include <string>
#include <map>
#include <iostream>
#include <fstream>

#include "inference_engine.hpp"
#include "vino_core_lib/vino_factory.h"
#include "vino_core_lib/models/attributes/base_attribute.h"

namespace Engines
{
class Engine;
}

namespace vino_core_lib
{
class ObjectDetectionResult;
class FaceDetectionResult;
}

namespace Models
{
/**
 * @class BaseModel
 * @brief This class represents the network given by .xml and .bin file
 */
class BaseModel : public ModelAttribute
{
public:
  using Ptr = std::shared_ptr<BaseModel>;

  BaseModel() {};

  /**
 * @brief Initialize the class with given .xml, .bin and .labels file. It will
 * also check whether the number of input and output are fit.
 * @param[in] model_loc The location of model' s .xml file
 * (model' s bin file should be the same as .xml file except for extension)
 * @param[in] input_num The number of input the network should have.
 * @param[in] output_num The number of output the network should have.
 * @param[in] batch_size The number of batch size (default: 1) the network should have.
 * @param[in] label_loc The location of label' s .label file
 * @return Whether the input device is successfully turned on.
 */
  BaseModel(const std::string& label_loc, const std::string& model_loc, int batch_size = 1);
  
  /**
 * @brief Initialize the class with given .xml, .bin and .labels file. It will
 * also check whether the number of input and output are fit.
 * @param[in] model_loc The location of model' s .xml file
 * (model' s bin file should be the same as .xml file except for extension)
 * @param[in] input_num The number of input the network should have.
 * @param[in] output_num The number of output the network should have.
 * @param[in] batch_size The number of batch size (default: 1) the network should have.
 * @param[in] label_loc The location of label' s .label file
 * @return Whether the input device is successfully turned on.
 */
  void init(const std::string& label_loc, const std::string& model_loc, int batch_size = 1);

  /**
 * @brief Get the maximum batch size of the model.
 * @return The maximum batch size of the model.
 */
  inline int getMaxBatchSize() const
  {
    return max_batch_size_;
  }
  inline void setMaxBatchSize(int max_batch_size)
  {
    max_batch_size_ = max_batch_size;
  }

  virtual bool enqueue(const std::shared_ptr<Engines::Engine>& engine, const cv::Mat& frame,
                       const cv::Rect& input_frame_loc)
  {
    return true;
  }
  /**
 * @brief Initialize the model. During the process the class will check
 * the network input, output size, check layer property and
 * set layer property.
 */
  void modelInit();
  /**
   * @brief Get the name of the model.
   * @return The name of the model.
   */
  virtual const std::string getModelCategory() const = 0;
  inline ModelAttr getAttribute()
  {
    return attr_;
  }

  inline InferenceEngine::CNNNetwork getNetReader() const
  {
    return net_reader_;
  }

protected:
  /**
   * New infterface to check and update Layer Property
   * @brief Set the layer property (layer layout, layer precision, etc.).
   * @param[in] network_reader The reader of the network to be set.
   */
  virtual bool updateLayerProperty(InferenceEngine::CNNNetwork& network_reader) = 0;

  InferenceEngine::Core engine;
  InferenceEngine::CNNNetwork net_reader_;
  void setFrameSize(const int& w, const int& h)
  {
    frame_size_.width = w;
    frame_size_.height = h;
  }
  cv::Size getFrameSize()
  {
    return frame_size_;
  }

private:
  int max_batch_size_;
  std::string model_loc_;
  std::string label_loc_;
  cv::Size frame_size_;
};

#define REG_MODEL_FACTORY         VinoFactory<std::string, Models::BaseModel>
#define REG_MODEL(BASE, key)  static REG_MODEL_FACTORY::TReg<Models::BASE> gs_model##_(key)

class ObjectDetectionModel : public BaseModel
{
public:
  ObjectDetectionModel() {};
  
  ObjectDetectionModel(const std::string& label_loc, const std::string& model_loc, int batch_size = 1);

  virtual bool fetchResults(const std::shared_ptr<Engines::Engine>& engine,
                            std::vector<vino_core_lib::ObjectDetectionResult>& result,
                            const float& confidence_thresh = 0.3, const bool& enable_roi_constraint = false) {};

  virtual bool matToBlob(const cv::Mat& orig_image, const cv::Rect&, float scale_factor, int batch_index,
                         const std::shared_ptr<Engines::Engine>& engine) = 0;
};

}  // namespace Models

#endif  // VINO_CORE_LIB__MODELS__BASE_MODEL_H
