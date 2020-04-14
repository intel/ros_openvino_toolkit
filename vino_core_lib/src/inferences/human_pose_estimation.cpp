// TODO add license

/**
 * @brief a header file with declaration of HumanPoseResult class //TODO update
 * @file human_pose_estimation.cpp
 * 
 * This file was mostly based on age_gender_detection.cpp. It also uses the code
 * from object_detection.cpp.
 */

#include <memory>
#include <string>

#include "vino_core_lib/inferences/human_pose_estimation.h"
#include "vino_core_lib/outputs/base_output.h"
#include "vino_core_lib/inferences/peak.h"

// HumanPoseResult
vino_core_lib::HumanPoseResult::HumanPoseResult(const cv::Rect& location)
    : Result(location)
{
}

vino_core_lib::HumanPoseResult::HumanPoseResult(
    const cv::Rect& location,
    const std::vector<cv::Point2f>& keypoints,
    const float& score)
    : keypoints(keypoints),
      score(score),
      Result(location)
{
}

// HumanPoseEstimation
vino_core_lib::HumanPoseEstimation::HumanPoseEstimation()
    : vino_core_lib::BaseInference(),
    upsampleRatio_(4),
    minPeaksDistance_(3.0f),
    midPointsScoreThreshold_(0.05f),
    foundMidPointsRatioThreshold_(0.8f),
    minJointsNumber_(3),
    minSubsetScore_(0.2f),
    stride_(8),
    pad_(cv::Vec4i::all(0))
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
  // object_detection.cpp
  if (width_ == 0 && height_ == 0) // TODO do we need this?
  {
    width_ = frame.cols;
    height_ = frame.rows;
  }

  // TODO verify this.
  imageSize_ = frame.size();
  
  //TODO implement preprocessing?

  if (!vino_core_lib::BaseInference::enqueue<u_int8_t>(
          frame, input_frame_loc, 1, 0, valid_model_->getInputName())) 
  {
    return false;
  }
  Result r(input_frame_loc);
  results_.clear();
  results_.emplace_back(r);
  return true;
}

bool vino_core_lib::HumanPoseEstimation::submitRequest()
{
  return vino_core_lib::BaseInference::submitRequest();
}

bool vino_core_lib::HumanPoseEstimation::fetchResults()
{
  bool can_fetch = vino_core_lib::BaseInference::fetchResults();
  if (!can_fetch) return false;
  auto request = getEngine()->getRequest();
  InferenceEngine::Blob::Ptr keypointsBlob =
      request->GetBlob(valid_model_->getOutputKeypointsName());
  InferenceEngine::Blob::Ptr heatmapBlob =
      request->GetBlob(valid_model_->getOutputHeatmapName());
  
  //TODO implement postprocessing function.

  // At this point, keypointsBlob has the [1,38,32,57] output
  
  results_.clear();
  CV_Assert(heatmapBlob->getTensorDesc().getDims()[1] == keypointsNumber_ + 1);
  InferenceEngine::SizeVector heatMapDims =
          heatmapBlob->getTensorDesc().getDims();
  //std::vector<Result> poses = postprocess(
  results_ = postprocess(
          heatmapBlob->buffer(),
          heatMapDims[2] * heatMapDims[3],
          keypointsNumber_,
          keypointsBlob->buffer(),
          heatMapDims[2] * heatMapDims[3],
          keypointsBlob->getTensorDesc().getDims()[1],
          heatMapDims[3], heatMapDims[2], imageSize_);
  
  slog::info << results_.size() << " people detected" << slog::endl;

  // for (size_t i = 0; i < results_.size(); ++i)
  // {
  //   results_[i].age_ = ageBlob->buffer().as<float*>()[i] * 100;
  //   results_[i].male_prob_ = genderBlob->buffer().as<float*>()[i * 2 + 1];
  // }
  return true;
}

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

using Result = vino_core_lib::HumanPoseResult;

std::vector<Result> vino_core_lib::HumanPoseEstimation::postprocess(
        const float* heatMapsData, const int heatMapOffset, const int nHeatMaps,
        const float* pafsData, const int pafOffset, const int nPafs,
        const int featureMapWidth, const int featureMapHeight,
        const cv::Size& imageSize) const
{
    std::vector<cv::Mat> heatMaps(nHeatMaps);
    for (size_t i = 0; i < heatMaps.size(); i++) {
        heatMaps[i] = cv::Mat(featureMapHeight, featureMapWidth, CV_32FC1,
                              reinterpret_cast<void*>(
                                  const_cast<float*>(
                                      heatMapsData + i * heatMapOffset)));
    }
    resizeFeatureMaps(heatMaps);

    std::vector<cv::Mat> pafs(nPafs);
    for (size_t i = 0; i < pafs.size(); i++) {
        pafs[i] = cv::Mat(featureMapHeight, featureMapWidth, CV_32FC1,
                          reinterpret_cast<void*>(
                              const_cast<float*>(
                                  pafsData + i * pafOffset)));
    }
    resizeFeatureMaps(pafs);

    std::vector<Result> poses = extractPoses(heatMaps, pafs);
    correctCoordinates(poses, heatMaps[0].size(), imageSize);
    return poses;
}

std::vector<Result> vino_core_lib::HumanPoseEstimation::extractPoses(
        const std::vector<cv::Mat>& heatMaps,
        const std::vector<cv::Mat>& pafs) const {
    std::vector<std::vector<human_pose_estimation::Peak> > peaksFromHeatMap(heatMaps.size());
    human_pose_estimation::FindPeaksBody findPeaksBody(heatMaps, minPeaksDistance_, peaksFromHeatMap);
    cv::parallel_for_(cv::Range(0, static_cast<int>(heatMaps.size())),
                      findPeaksBody);
    int peaksBefore = 0;
    for (size_t heatmapId = 1; heatmapId < heatMaps.size(); heatmapId++) {
        peaksBefore += static_cast<int>(peaksFromHeatMap[heatmapId - 1].size());
        for (auto& peak : peaksFromHeatMap[heatmapId]) {
            peak.id += peaksBefore;
        }
    }
    std::vector<Result> poses = groupPeaksToPoses(
                peaksFromHeatMap, pafs, keypointsNumber_, midPointsScoreThreshold_,
                foundMidPointsRatioThreshold_, minJointsNumber_, minSubsetScore_);
    return poses;
}

void vino_core_lib::HumanPoseEstimation::resizeFeatureMaps(std::vector<cv::Mat>& featureMaps) const {
    for (auto& featureMap : featureMaps) {
        cv::resize(featureMap, featureMap, cv::Size(),
                   upsampleRatio_, upsampleRatio_, cv::INTER_CUBIC);
    }
}

void vino_core_lib::HumanPoseEstimation::correctCoordinates(std::vector<Result>& poses,
                                            const cv::Size& featureMapsSize,
                                            const cv::Size& imageSize) const {
    CV_Assert(stride_ % upsampleRatio_ == 0);

    cv::Size fullFeatureMapSize = featureMapsSize * stride_ / upsampleRatio_;

    float scaleX = imageSize.width /
            static_cast<float>(fullFeatureMapSize.width - pad_(1) - pad_(3));
    float scaleY = imageSize.height /
            static_cast<float>(fullFeatureMapSize.height - pad_(0) - pad_(2));
    for (auto& pose : poses) {
        for (auto& keypoint : pose.keypoints) {
            if (keypoint != cv::Point2f(-1, -1)) {
                keypoint.x *= stride_ / upsampleRatio_;
                keypoint.x -= pad_(1);
                keypoint.x *= scaleX;

                keypoint.y *= stride_ / upsampleRatio_;
                keypoint.y -= pad_(0);
                keypoint.y *= scaleY;
            }
        }
    }
}

