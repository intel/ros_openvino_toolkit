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
 * @brief a header file with declaration of PersonReidentificationModel class
 * @file person_reidentification_model.cpp
 */
#include <string>
#include "vino_core_lib/models/person_reidentification_model.h"
#include "vino_core_lib/slog.h"
// Validated Object Detection Network
Models::PersonReidentificationModel::PersonReidentificationModel(const std::string& label_loc, const std::string& model_loc, int max_batch_size)
  : BaseModel(label_loc, model_loc, max_batch_size)
{
}

bool Models::PersonReidentificationModel::updateLayerProperty(InferenceEngine::CNNNetwork& netreader)
{
  slog::info << "Checking Inputs for Model" << getModelName() << slog::endl;

  auto network = netreader;

  InferenceEngine::InputsDataMap input_info_map(network.getInputsInfo());

  InferenceEngine::InputInfo::Ptr input_info = input_info_map.begin()->second;
  input_info->setPrecision(InferenceEngine::Precision::U8);
  input_info->getInputData()->setLayout(InferenceEngine::Layout::NCHW);
  // set output property
  InferenceEngine::OutputsDataMap output_info_map(network.getOutputsInfo());
  // set input and output layer name
  input_ = input_info_map.begin()->first;
  output_ = output_info_map.begin()->first;

  return true;
}

const std::string Models::PersonReidentificationModel::getModelCategory() const
{
  return "Person Reidentification";
}

REG_MODEL(PersonReidentificationModel, "PersonReidentification");
