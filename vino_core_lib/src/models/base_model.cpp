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
 * @brief a header file with declaration of BaseModel class
 * @file base_model.cpp
 */

#include <algorithm>
#include <fstream>
#include <memory>
#include <string>

#include "vino_core_lib/models/base_model.h"
#include "vino_core_lib/slog.h"

// Validated Base Network
Models::BaseModel::BaseModel(const std::string& label_loc, const std::string& model_loc, int max_batch_size)
  : label_loc_(label_loc), model_loc_(model_loc), max_batch_size_(max_batch_size), ModelAttribute(model_loc)
{
  slog::debug << "model location:  " << model_loc_ << slog::endl;
  if (model_loc.empty())
  {
    throw std::logic_error("model file name is empty!");
  }

  ///net_reader_ = std::make_shared<InferenceEngine::CNNNetReader>();
}

void Models::BaseModel::init(const std::string& label_loc, const std::string& model_loc, int max_batch_size)
{
  label_loc_ = label_loc;
  model_loc_ = model_loc;
  max_batch_size_ = max_batch_size;
  attr_.model_name = model_loc;
  // TODO
  // ModelAttribute(model_loc);

  slog::debug << "model location:  " << model_loc_ << slog::endl;

  if (model_loc.empty())
  {
    throw std::logic_error("model file name is empty!");
  }

  modelInit();

  ///net_reader_ = std::make_shared<InferenceEngine::CNNNetReader>();
}

void Models::BaseModel::modelInit()
{
  slog::info << "Loading network files: " << model_loc_ << slog::endl;

  // Read network model
  ///net_reader_->ReadNetwork(model_loc_);
  net_reader_ = engine.ReadNetwork(model_loc_);
  // Extract model name and load it's weights
  // remove extension
  size_t last_index = model_loc_.find_last_of(".");
  std::string raw_name = model_loc_.substr(0, last_index);
  ///std::string bin_file_name = raw_name + ".bin";
  ///net_reader_->ReadWeights(bin_file_name);
  // Read labels (if any)
  std::string label_file_name = label_loc_.substr(0, last_index);
  //std::string label_file_name = raw_name + ".labels";
  loadLabelsFromFile(label_file_name);

  // Set batch size to given max_batch_size_
  slog::info << "Batch size is set to  " << max_batch_size_ << slog::endl;
  net_reader_.setBatchSize(max_batch_size_);

  updateLayerProperty(net_reader_);
}

#if 0
bool Models::BaseModel::updateLayerProperty(
  InferenceEngine::CNNNetReader::Ptr net_reader)
{
#if 0
  if (!updateLayerProperty(net_reader)){
    slog::warn << "The model(name: " << getModelName() << ") failed to update Layer Property!"
      << slog::endl;
    return false;
  }
#endif
  if(!isVerified()){
    slog::warn << "The model(name: " << getModelName() << ") does NOT pass Attribute Check!"
      << slog::endl;
    return false;
  }

  return true;
}
#endif

Models::ObjectDetectionModel::ObjectDetectionModel(const std::string& label_loc, const std::string& model_loc, int max_batch_size)
  : BaseModel(label_loc, model_loc, max_batch_size)
{
}
