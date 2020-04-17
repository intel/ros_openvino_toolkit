// TODO add license

/*
Model URL: https://docs.openvinotoolkit.org/2019_R3.1/_models_intel_human_pose_estimation_0001_description_human_pose_estimation_0001.html
*/

#include <string>
#include <sstream>

#include "vino_core_lib/models/human_pose_estimation_model.h"
#include "vino_core_lib/slog.h"

Models::HumanPoseEstimationModel::HumanPoseEstimationModel(
                                               const std::string& model_loc,
                                               int input_num, int output_num,
                                               int max_batch_size)
    : BaseModel(model_loc, input_num, output_num, max_batch_size)
{
}

void Models::HumanPoseEstimationModel::setLayerProperty(
    InferenceEngine::CNNNetReader::Ptr net_reader)
{
  // set input property
  InferenceEngine::InputsDataMap input_info_map(
  net_reader->getNetwork().getInputsInfo());
  InferenceEngine::InputInfo::Ptr input_info = input_info_map.begin()->second;
  input_info->setPrecision(InferenceEngine::Precision::U8); // It's an image
  input_info->setLayout(InferenceEngine::Layout::NCHW);     // No idea.
  // set output property
  InferenceEngine::OutputsDataMap output_info_map(
      net_reader->getNetwork().getOutputsInfo());
  auto it = output_info_map.begin();
  InferenceEngine::DataPtr keypoints_output_ptr = (it++)->second;
  InferenceEngine::DataPtr heatmap_output_ptr = (it++)->second;
  keypoints_output_ptr->setPrecision(InferenceEngine::Precision::FP32);
  keypoints_output_ptr->setLayout(InferenceEngine::Layout::NCHW);
  heatmap_output_ptr->setPrecision(InferenceEngine::Precision::FP32);
  heatmap_output_ptr->setLayout(InferenceEngine::Layout::NCHW);
  // set input and output layer name
  input_ = input_info_map.begin()->first;
  output_keypoints_ = keypoints_output_ptr->name;
  output_heatmap_ = heatmap_output_ptr->name;
}

void Models::HumanPoseEstimationModel::checkLayerProperty(
    const InferenceEngine::CNNNetReader::Ptr& net_reader)
{
  slog::info << "Checking Human Pose Estimation outputs" << slog::endl;
  InferenceEngine::OutputsDataMap output_info(
      net_reader->getNetwork().getOutputsInfo());
  auto it = output_info.begin();
  InferenceEngine::DataPtr paf_output_ptr = (it++)->second;
  InferenceEngine::DataPtr heatmap_output_ptr = (it++)->second;
  
  if (paf_output_ptr->getCreatorLayer().lock()->type != "Convolution")
  {
    throw std::logic_error("In Human Pose Estimation network, PAF layer (" +
                           paf_output_ptr->getCreatorLayer().lock()->name +
                           ") should be a Convolution, but was: " +
                           paf_output_ptr->getCreatorLayer().lock()->type);
  }
  if (heatmap_output_ptr->getCreatorLayer().lock()->type != "Convolution")
  {
    throw std::logic_error("In Human Pose Estimation network, heatmap layer (" +
                           heatmap_output_ptr->getCreatorLayer().lock()->name +
                           ") should be a Convolution, but was: " +
                           heatmap_output_ptr->getCreatorLayer().lock()->type);
  }

  if (paf_output_ptr->getCreatorLayer().lock()->outData.size() != 1)
  {
    throw std::logic_error(
      "In Human Pose Estimation network, PAF layer (" +
      paf_output_ptr->getCreatorLayer().lock()->name +
      ") should have 1 output, but had: " +
      std::to_string(paf_output_ptr->getCreatorLayer().lock()->outData.size()));
  }
  if (heatmap_output_ptr->getCreatorLayer().lock()->outData.size() != 1)
  {
    throw std::logic_error(
      "In Human Pose Estimation network, Heatmap layer (" +
      heatmap_output_ptr->getCreatorLayer().lock()->name +
      ") should have 1 output, but had: " +
      std::to_string(heatmap_output_ptr->getCreatorLayer().lock()->outData.size()));
  }

  if (paf_output_ptr->getCreatorLayer().lock()->outData[0]->dims.size() == 4 && 
      paf_output_ptr->getCreatorLayer().lock()->outData[0]->dims[2] == 19)
  {
    std::swap(paf_output_ptr, heatmap_output_ptr);
  }

  auto pafDims = paf_output_ptr->getCreatorLayer().lock()->outData[0]->dims;
  auto heatDims = heatmap_output_ptr->getCreatorLayer().lock()->outData[0]->dims;

  if (pafDims.size() != 4 || pafDims[0] != 57 || pafDims[1] != 32
      || pafDims[2] != 38 || pafDims[3] != 1)
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
      paf_output_ptr->getCreatorLayer().lock()->name +
      ") should have output size of [ 57 32 38 1 ], but had: " + size.str());
  }

  if (heatDims.size() != 4 || heatDims[0] != 57 || heatDims[1] != 32
      || heatDims[2] != 19 || heatDims[3] != 1)
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
             << paf_output_ptr->getCreatorLayer().lock()->name
             << slog::endl;
  slog::info << "Heatmap layer: "
             << heatmap_output_ptr->getCreatorLayer().lock()->name 
             << slog::endl;
}

const std::string Models::HumanPoseEstimationModel::getModelName() const
{
  return "Human Pose Estimation";
}