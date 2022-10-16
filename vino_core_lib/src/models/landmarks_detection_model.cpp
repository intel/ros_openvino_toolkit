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
Models::LandmarksDetectionModel::LandmarksDetectionModel(const std::string& label_loc, const std::string& model_loc, int max_batch_size)
  : BaseModel(label_loc, model_loc, max_batch_size)
{
}
 
bool Models::LandmarksDetectionModel::updateLayerProperty(std::shared_ptr<ov::Model>& net_reader)
{
  //INPUT
  auto inputs_info_map = net_reader->inputs();
  ov::preprocess::PrePostProcessor ppp = ov::preprocess::PrePostProcessor(net_reader);
  std::string input_tensor_name_ = net_reader->input().get_any_name();
  ov::preprocess::InputInfo& input_info = ppp.input(input_tensor_name_);
 
  auto input_layerName = inputs_info_map[0];
  auto input_layerData = inputs_info_map[1];
  addInputInfo("input_layerName", input_layerName.get_any_name());
  addInputInfo("input_layerData", input_layerData.get_any_name());
  ov::Shape input_layerDims = input_layerData.get_shape();

  if (input_layerDims.size() == 4) 
  {
      const ov::Layout tensor_layout{"NCHW"};
      ppp.input(input_layerData.get_any_name()).tensor().
          set_element_type(ov::element::u8).
          set_layout(tensor_layout);
  } 
  else if (input_layerDims.size() == 2) 
  {
      const ov::Layout tensor_layout{"NC"};
      ppp.input(input_layerData.get_any_name()).tensor().
          set_element_type(ov::element::f32).
          set_layout(tensor_layout);
  } 
  else 
  {
      throw std::runtime_error("Unknow type of input layer layout. Expected either 4 or 2 dimensional inputs");
  }

  // OUTPUT
  std::string output_tensor_name = net_reader->output().get_any_name();
  ov::preprocess::OutputInfo& output_info = ppp.output(output_tensor_name);
  auto outputs_info_map = net_reader->outputs();  
  auto output_layerName = outputs_info_map[0];
  auto output_layerData = outputs_info_map[1];
  ov::Shape output_layerDims = output_layerData.get_shape();
  ppp.output(output_layerData.get_any_name()).tensor().set_element_type(ov::element::f32);

  net_reader = ppp.build();
  addOutputInfo("output_layerName", output_layerName.get_any_name());
  addOutputInfo("output_layerData", output_layerData.get_any_name());
  return true;
}

const std::string Models::LandmarksDetectionModel::getModelCategory() const
{
  return "Landmarks Detection";
}
