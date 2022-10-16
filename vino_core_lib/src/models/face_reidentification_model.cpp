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
#include <string>
#include "vino_core_lib/models/face_reidentification_model.h"
#include "vino_core_lib/slog.h"
// Validated Face Reidentification Network
Models::FaceReidentificationModel::FaceReidentificationModel(const std::string& label_loc, const std::string& model_loc, int max_batch_size)
  : BaseModel(label_loc, model_loc, max_batch_size)
{
}

bool Models::FaceReidentificationModel::updateLayerProperty(std::shared_ptr<ov::Model>& net_reader)
{
  // set input property
  auto inputs_info_map = net_reader->inputs();
  ov::preprocess::PrePostProcessor ppp = ov::preprocess::PrePostProcessor(net_reader);
  std::string input_tensor_name_ = net_reader->input().get_any_name();
  ov::preprocess::InputInfo& input_info = ppp.input(input_tensor_name_);

  const ov::Layout tensor_layout{"NCHW"};
  input_info.tensor().
    set_element_type(ov::element::u8).
    set_layout(tensor_layout);
  addInputInfo("input", input_tensor_name_);
  
  // set output property
  auto outputs_info = net_reader->outputs();
  std::string output_tensor_name = net_reader->output().get_any_name();
  ov::preprocess::OutputInfo& output_info = ppp.output(output_tensor_name);
  output_info.tensor().set_element_type(ov::element::f32);
  addOutputInfo("output", output_tensor_name);
  net_reader = ppp.build();

  return true;
}

const std::string Models::FaceReidentificationModel::getModelCategory() const
{
  return "Face Reidentification";
}
