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

#include <string>
#include <sstream>

#include "dynamic_vino_lib/models/human_pose_estimation_model.h"
#include "dynamic_vino_lib/slog.h"

// Validated Human Pose Network
Models::HumanPoseEstimationModel::HumanPoseEstimationModel(const std::string& model_loc, int max_batch_size)
  : BaseModel(model_loc, max_batch_size)
{
}

bool Models::HumanPoseEstimationModel::updateLayerProperty(InferenceEngine::CNNNetReader::Ptr net_reader)
{
  slog::info << "Checking INPUTs for model " << getModelName() << slog::endl;
  // set input property
  InferenceEngine::InputsDataMap input_info_map(net_reader->getNetwork().getInputsInfo());
  if (input_info_map.size() != 1)
  {
    slog::warn << "This model should have only one input, but we got" << std::to_string(input_info_map.size())
               << "inputs" << slog::endl;
    return false;
  }
  InferenceEngine::InputInfo::Ptr input_info = input_info_map.begin()->second;
  input_info->setPrecision(InferenceEngine::Precision::U8);
  input_info->getInputData()->setLayout(InferenceEngine::Layout::NCHW);
  addInputInfo("input", input_info_map.begin()->first);

  InferenceEngine::OutputsDataMap output_info_map(net_reader->getNetwork().getOutputsInfo());

  auto it = output_info_map.begin();
  InferenceEngine::DataPtr keypoints_output_ptr = (it++)->second;
  InferenceEngine::DataPtr heatmap_output_ptr = (it++)->second;
  keypoints_output_ptr->setPrecision(InferenceEngine::Precision::FP32);
  keypoints_output_ptr->setLayout(InferenceEngine::Layout::NCHW);
  heatmap_output_ptr->setPrecision(InferenceEngine::Precision::FP32);
  heatmap_output_ptr->setLayout(InferenceEngine::Layout::NCHW);
  // set input and output layer name
  input_ = input_info_map.begin()->first;

  output_keypoints_ = keypoints_output_ptr->getName();
  output_heatmap_ = heatmap_output_ptr->getName();
//   addOutputInfo("output_keypoints_", keypoints_output_ptr->getName());
//   addOutputInfo("output_heatmap_", keypoints_output_ptr->getName());

//   it = output_info_map.begin();
//   InferenceEngine::DataPtr keypoints_output_ptr = (it++)->second;
//   InferenceEngine::DataPtr heatmap_output_ptr = (it++)->second;

  if (keypoints_output_ptr->getCreatorLayer().lock()->type != "Convolution")
  {
    throw std::logic_error("In Human Pose Estimation network, PAF layer (" +
                           keypoints_output_ptr->getCreatorLayer().lock()->name +
                           ") should be a Convolution, but was: " +
                           keypoints_output_ptr->getCreatorLayer().lock()->type);
  }
  if (heatmap_output_ptr->getCreatorLayer().lock()->type != "Convolution")
  {
    throw std::logic_error("In Human Pose Estimation network, heatmap layer (" +
                           heatmap_output_ptr->getCreatorLayer().lock()->name +
                           ") should be a Convolution, but was: " +
                           heatmap_output_ptr->getCreatorLayer().lock()->type);
  }

  if (keypoints_output_ptr->getCreatorLayer().lock()->outData.size() != 1)
  {
    throw std::logic_error(
      "In Human Pose Estimation network, PAF layer (" +
      keypoints_output_ptr->getCreatorLayer().lock()->name +
      ") should have 1 output, but had: " +
      std::to_string(keypoints_output_ptr->getCreatorLayer().lock()->outData.size()));
  }
  if (heatmap_output_ptr->getCreatorLayer().lock()->outData.size() != 1)
  {
    throw std::logic_error(
      "In Human Pose Estimation network, Heatmap layer (" +
      heatmap_output_ptr->getCreatorLayer().lock()->name +
      ") should have 1 output, but had: " +
      std::to_string(heatmap_output_ptr->getCreatorLayer().lock()->outData.size()));
  }

  if (keypoints_output_ptr->getCreatorLayer().lock()->outData[0]->getDims().size() == 4 && 
      keypoints_output_ptr->getCreatorLayer().lock()->outData[0]->getDims()[2] == 19)
  {
    std::swap(keypoints_output_ptr, heatmap_output_ptr);
  }

  auto pafDims = keypoints_output_ptr->getCreatorLayer().lock()->outData[0]->getDims();
  auto heatDims = heatmap_output_ptr->getCreatorLayer().lock()->outData[0]->getDims();

  // if (pafDims.size() != 4 || pafDims[0] != 57 || pafDims[1] != 32
  //     || pafDims[2] != 38 || pafDims[3] != 1)  
  if (pafDims.size() != 4 || pafDims[0] != 1 || pafDims[1] != 38
      || pafDims[2] != 32 || pafDims[3] != 57)
  {
    std::ostringstream size;
    size << "[ ";
    for (size_t s : pafDims)
    {
      size << s << " ";
    }
    size << "]";
    throw std::logic_error(
      "In Human Pose Estimation network, PAF layer (" +
      keypoints_output_ptr->getCreatorLayer().lock()->name +
      ") should have output size of [ 57 32 38 1 ], but had: " + size.str());
  }

  // if (heatDims.size() != 4 || heatDims[0] != 57 || heatDims[1] != 32
  //     || heatDims[2] != 19 || heatDims[3] != 1)
  if (heatDims.size() != 4 || heatDims[0] != 1 || heatDims[1] != 19
      || heatDims[2] != 32 || heatDims[3] != 57)
  {
    std::ostringstream size;
    size << "[ ";
    for (size_t s : heatDims)
    {
      size << s << " ";
    }
    size << "]";
    throw std::logic_error(
      "In Human Pose Estimation network, Heatmap layer (" +
      heatmap_output_ptr->getCreatorLayer().lock()->name +
      ") should have output size of [ 57 32 19 1 ], but had: " + size.str());
  }

  slog::info << "PAF layer: " 
             << keypoints_output_ptr->getCreatorLayer().lock()->name
             << slog::endl;
  slog::info << "Heatmap layer: "
             << heatmap_output_ptr->getCreatorLayer().lock()->name 
             << slog::endl;

  printAttribute();
  return true;
}

const std::string Models::HumanPoseEstimationModel::getModelCategory() const
{
  return "Head Pose Network";
}