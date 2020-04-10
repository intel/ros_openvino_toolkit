// TODO add license

// TODO Remove these comments
/*
Model URL: https://docs.openvinotoolkit.org/latest/_models_intel_human_pose_estimation_0001_description_human_pose_estimation_0001.html

# Inputs

Shape: [1x3x256x456]. An input image in the [BxCxHxW] format , where:
    B - batch size
    C - number of channels
    H - image height
    W - image width. Expected color order is BGR.

# Outputs

The net outputs two blobs with the [1, 38, 32, 57] and [1, 19, 32, 57] shapes. 
The first blob contains keypoint pairwise relations (part affinity fields), 
while the second blob contains keypoint heatmaps.
*/

#include <string>

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
  slog::info << "Checking Age Gender Detection outputs" << slog::endl;
  InferenceEngine::OutputsDataMap output_info(
      net_reader->getNetwork().getOutputsInfo());
  auto it = output_info.begin();
  InferenceEngine::DataPtr keypoints_output_ptr = (it++)->second;
  InferenceEngine::DataPtr heatmap_output_ptr = (it++)->second;
  // output layer of age should be Convolution type
  // TODO check if the output types are correct.
//   if (heatmap_output_ptr->getCreatorLayer().lock()->type == "Convolution")
//   {
//     std::swap(keypoints_output_ptr, heatmap_output_ptr);
//   }
//   if (keypoints_output_ptr->getCreatorLayer().lock()->type != "Convolution")
//   {
//     throw std::logic_error("In Age Gender network, age layer (" +
//                            keypoints_output_ptr->getCreatorLayer().lock()->name +
//                            ") should be a Convolution, but was: " +
//                            keypoints_output_ptr->getCreatorLayer().lock()->type);
//   }
//   if (heatmap_output_ptr->getCreatorLayer().lock()->type != "SoftMax")
//   {
//     throw std::logic_error("In Age Gender network, gender layer (" +
//                            heatmap_output_ptr->getCreatorLayer().lock()->name +
//                            ") should be a SoftMax, but was: " +
//                            heatmap_output_ptr->getCreatorLayer().lock()->type);
//   }
  slog::info << "Keypoints layer: " 
             << keypoints_output_ptr->getCreatorLayer().lock()->name
             << slog::endl;
  slog::info << "Heatmap layer: "
             << heatmap_output_ptr->getCreatorLayer().lock()->name 
             << slog::endl;
}

const std::string Models::HumanPoseEstimationModel::getModelName() const
{
  return "Human Pose Estimation";
}