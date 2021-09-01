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
 * @brief a header file with declaration of LandmarksDetectionModel class
 * @file landmarks_detection_model.cpp
 */
#include <string>
#include "vino_core_lib/models/landmarks_detection_model.h"
#include "vino_core_lib/slog.h"
// Validated Landmarks Detection Network
Models::LandmarksDetectionModel::LandmarksDetectionModel(const std::string& model_loc, int max_batch_size)
  : BaseModel(model_loc, max_batch_size)
{
}

bool Models::LandmarksDetectionModel::updateLayerProperty(InferenceEngine::CNNNetwork& net_reader)
{
  //INPUT
  InferenceEngine::InputsDataMap input_info_map(net_reader.getInputsInfo());
  auto input_layerName = input_info_map.begin()->first;
  auto input_layerData = input_info_map.begin()->second;
  auto input_layerDims = input_layerData->getTensorDesc().getDims();

  if (input_layerDims.size() == 4) 
  {
      input_layerData->setLayout(InferenceEngine::Layout::NCHW);
      input_layerData->setPrecision(InferenceEngine::Precision::U8);
  } 
  else if (input_layerDims.size() == 2) 
  {
      input_layerData->setLayout(InferenceEngine::Layout::NC);
      input_layerData->setPrecision(InferenceEngine::Precision::FP32);
  } 
  else 
  {
      throw std::runtime_error("Unknow type of input layer layout. Expected either 4 or 2 dimensional inputs");
  }

  // OUTPUT
  InferenceEngine::OutputsDataMap output_info_map(net_reader.getOutputsInfo());
  auto output_layerName = output_info_map.begin()->first;
  auto output_layerData = output_info_map.begin()->second;
  auto output_layerDims = output_layerData->getTensorDesc().getDims();

  output_layerData->setPrecision(InferenceEngine::Precision::FP32);
  // output_layerData->setLayout(InferenceEngine::Layout::NCHW);
  // set input and output layer name
  input_ = input_layerName;
  output_ = output_layerName;
  return true;
}

const std::string Models::LandmarksDetectionModel::getModelCategory() const
{
  return "Landmarks Detection";
}
