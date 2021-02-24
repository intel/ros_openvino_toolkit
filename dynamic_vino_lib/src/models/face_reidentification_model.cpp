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
 * @brief a header file with declaration of FaceReidentificationModel class
 * @file face_reidentification_model.cpp
 */
#include "dynamic_vino_lib/models/face_reidentification_model.h"
#include "dynamic_vino_lib/slog.h"
#include <string>
// Validated Face Reidentification Network
Models::FaceReidentificationModel::FaceReidentificationModel(const std::string& model_loc, int max_batch_size)
  : BaseModel(model_loc, max_batch_size)
{
}

void Models::FaceReidentificationModel::setLayerProperty(InferenceEngine::CNNNetReader::Ptr net_reader)
{
  // set input property
  InferenceEngine::InputsDataMap input_info_map(net_reader->getNetwork().getInputsInfo());
  InferenceEngine::InputInfo::Ptr input_info = input_info_map.begin()->second;
  input_info->setPrecision(InferenceEngine::Precision::U8);
  input_info->getInputData()->setLayout(InferenceEngine::Layout::NCHW);
  // set output property
  InferenceEngine::OutputsDataMap output_info_map(net_reader->getNetwork().getOutputsInfo());
  InferenceEngine::DataPtr& output_data_ptr = output_info_map.begin()->second;
  output_data_ptr->setPrecision(InferenceEngine::Precision::FP32);
  output_data_ptr->setLayout(InferenceEngine::Layout::NCHW);
  // set input and output layer name
  input_ = input_info_map.begin()->first;
  output_ = output_info_map.begin()->first;
}

void Models::FaceReidentificationModel::checkLayerProperty(const InferenceEngine::CNNNetReader::Ptr& net_reader)
{
}

const std::string Models::FaceReidentificationModel::getModelCategory() const
{
  return "Face Reidentification";
}
