// TODO add license

/**
 * @brief a header file with declaration of HumanPoseResult class //TODO update
 * @file human_pose_estimation.cpp
 */

#include <memory>
#include <string>

#include "vino_core_lib/inferences/human_pose_estimation.h"
#include "vino_core_lib/outputs/base_output.h"

// HumanPoseResult
vino_core_lib::HumanPoseResult::HumanPoseResult(const cv::Rect& location)
    : Result(location)
{
}

// HumanPoseEstimation
vino_core_lib::HumanPoseEstimation::HumanPoseEstimation()
    : vino_core_lib::BaseInference()
{
}

vino_core_lib::HumanPoseEstimation::~HumanPoseEstimation() = default;

void vino_core_lib::HumanPoseEstimation::loadNetwork(
    std::shared_ptr<Models::HumanPoseEstimationModel> network)
{
  valid_model_ = network;
  setMaxBatchSize(network->getMaxBatchSize());
}

bool vino_core_lib::HumanPoseEstimation::enqueue(
    const cv::Mat& frame, const cv::Rect& input_frame_loc)
{
  if (getEnqueuedNum() == 0)
  {
    results_.clear();
  }
  bool succeed = vino_core_lib::BaseInference::enqueue<float>(
      frame, input_frame_loc, 1, getResultsLength(),
      valid_model_->getInputName());
  if (!succeed) return false;
  Result r(input_frame_loc);
  results_.emplace_back(r);
  return true;
}

bool vino_core_lib::HumanPoseEstimation::submitRequest()
{
  return vino_core_lib::BaseInference::submitRequest();
}

// TODO fetchResults()

const int vino_core_lib::HumanPoseEstimation::getResultsLength() const
{
  return static_cast<int>(results_.size());
}

const vino_core_lib::Result*
vino_core_lib::HumanPoseEstimation::getLocationResult(int idx) const
{
  return &(results_[idx]);
}

const std::string vino_core_lib::HumanPoseEstimation::getName() const
{
  return valid_model_->getModelName();
}

const void vino_core_lib::HumanPoseEstimation::observeOutput(
    const std::shared_ptr<Outputs::BaseOutput>& output)
{
  if (output != nullptr)
  {
    output->accept(results_);
  }
}
