// TODO add license

/**
 * @brief A header file with declaration for AgeGenderDetection Class
 * @file human_pose_estimation.h
 */
#ifndef DYNAMIC_VINO_LIB_INFERENCES_HUMAN_POSE_ESTIMATION_H
#define DYNAMIC_VINO_LIB_INFERENCES_HUMAN_POSE_ESTIMATION_H

#include <memory>
#include <string>
#include <vector>

#include "dynamic_vino_lib/engines/engine.h"
#include "dynamic_vino_lib/inferences/base_inference.h"
#include "dynamic_vino_lib/models/human_pose_estimation_model.h"
#include "inference_engine.hpp"
#include "opencv2/opencv.hpp"

namespace dynamic_vino_lib
{

class HumanPoseKeypoint : public cv::Point2f
{
public:
  HumanPoseKeypoint(float x, float y, float score)
    : HumanPoseKeypoint::Point_(x, y),
      score(score) {};
  HumanPoseKeypoint(cv::Point2f point)
    : HumanPoseKeypoint::Point_(point),
      score(score) {};
  float score;
};

/**
 * @class HumanPoseResult
 * @brief Class for storing and processing age and gender detection result.
 */
class HumanPoseResult : public Result
{
 public:
  friend class HumanPoseEstimation;
  explicit HumanPoseResult(const cv::Rect& location);
  explicit HumanPoseResult(
    const cv::Rect& location,
    const std::vector<HumanPoseKeypoint>& keypoints, // = std::vector<cv::Point2f>(),
    const float& score); // = 0);
  
  // Following similar structure of dynamic_vino_lib/inferences/object_detection.h
  // and human_pose_estimation_demo/src/human_pose_estimator.h

  /**
   * @brief Get the age keypoints of the estimated pose from the result.
   * @return The estimated keypoints.
   */
  std::vector<HumanPoseKeypoint> getKeypoints() const
  {
    return keypoints;
  }

  /**
   * @brief Get the score of the estimated pose from the result.
   * @return The score of the estimation.
   */
  float getScore() const
  {
    return score;
  }

  std::vector<float> keypointsScores;
  std::vector<HumanPoseKeypoint> keypoints;
  float score = -1;
};


/**
 * @class HumanPoseEstimation
 * @brief Class to load the human pose estimation model and perform
   human pose estimation.
 */
class HumanPoseEstimation : public BaseInference
{
 public:
  using Result = dynamic_vino_lib::HumanPoseResult;
  HumanPoseEstimation();
  ~HumanPoseEstimation() override;
  /**
   * @brief Load the age gender detection model.
   */
  void loadNetwork(std::shared_ptr<Models::HumanPoseEstimationModel>);
  /**
   * @brief Enqueue a frame to this class.
   * The frame will be buffered but not inferred yet.
   * @param[in] frame The frame to be enqueued.
   * @param[in] input_frame_loc The location of the enqueued frame with respect
   * to the frame generated by the input device.
   * @return Whether this operation is successful.
   */
  bool enqueue(const cv::Mat& frame, const cv::Rect&) override;
  /**
   * @brief Start inference for all buffered frames.
   * @return Whether this operation is successful.
   */
  bool submitRequest() override;
  /**
   * @brief This function will fetch the results of the previous inference and
   * stores the results in a result buffer array. All buffered frames will be
   * cleared.
   * @return Whether the Inference object fetches a result this time
   */
  bool fetchResults() override;
  /**
   * @brief Get the length of the buffer result array.
   * @return The length of the buffer result array.
   */
  int getResultsLength() const override;
  /**
   * @brief Get the location of result with respect
   * to the frame generated by the input device.
   * @param[in] idx The index of the result.
   */
  const dynamic_vino_lib::Result* getLocationResult(int idx) const override;
  /**
   * @brief Get the name of the Inference instance.
   * @return The name of the Inference instance.
   */
  const std::string getName() const override;
  /**
   * @brief Show the observed detection result either through image window
   * or ROS topic.
   */
  void observeOutput(const std::shared_ptr<Outputs::BaseOutput>& output) override;
  
  const std::vector<cv::Rect> getFilteredROIs(const std::string filter_conditions) const override;

  /**
   * @brief Processess the network's outputs to extract the valid poses.
   * 
   * Copied from: https://github.com/opencv/open_model_zoo/blob/master/demos/human_pose_estimation_demo/src/human_pose_estimator.cpp
   * 
   * @param heatMapsData Data outputted by the heatmap network.
   * @param heatMapOffset Size of each heatmap result.
   * @param nHeatMaps Number of keypoints.
   * @param pafsData Data outputted by the PAF network.
   * @param pafOffset Size of each PAF result.
   * @param nPafs Numver of PAFs.
   * @param featureMapWidth Width of the heatmap.
   * @param featureMapHeight Height of the heatmap.
   * @param imageSize Size of the input image.
   * @return std::vector<Result> A vector with the detected poses.
   */
  std::vector<Result> postprocess(
            const float* heatMapsData, const int heatMapOffset, const int nHeatMaps,
            const float* pafsData, const int pafOffset, const int nPafs,
            const int featureMapWidth, const int featureMapHeight,
            const cv::Size& imageSize) const;
            
 private:
  
  /**
   * @brief Resizes the heatmap by upSampleRatio.
   * 
   * @param featureMaps A vector with the heatmaps to resize.
   */
  void resizeFeatureMaps(std::vector<cv::Mat>& featureMaps) const;

  /**
   * @brief Extracts the poses from the given heatmaps and PAFs.
   * 
   * @param heatMaps Postprocessed heatmaps.
   * @param pafs Postprocessed PAFs.
   * @return std::vector<Result> The detected poses.
   */
  std::vector<Result> extractPoses(
        const std::vector<cv::Mat>& heatMaps,
        const std::vector<cv::Mat>& pafs) const;

  /**
   * @brief Aligns the poses' keypoints to the input image.
   * 
   * @param poses Poses (extracted from the heatmaps and PAFs).
   * @param featureMapsSize  The size of the heatmaps.
   * @param imageSize The input image size.
   */
  void correctCoordinates(std::vector<Result>& poses,
                          const cv::Size& featureMapsSize,
                          const cv::Size& imageSize) const;
  
  /**
   * @brief Correct the bonding boxes based on the poses' keypoints.
   * 
   * @param poses Poses (with corrected keypoints).
   */
  void correctROI(std::vector<Result>& poses) const;

  std::shared_ptr<Models::HumanPoseEstimationModel> valid_model_;
  std::vector<Result> results_;
  int width_ = 0;
  int height_ = 0;

  const size_t keypointsNumber_ = 18;
  int upsampleRatio_ = 4;
  float minPeaksDistance_ = 3.0;
  float midPointsScoreThreshold_ = 0.05;
  float foundMidPointsRatioThreshold_ = 0.8;
  int minJointsNumber_ = 3;
  float minSubsetScore_ = 0.2;
  int stride_ = 8;
  cv::Vec4i pad_ = cv::Vec4i::all(0);
};

} //  namespace dynamic_vino_lib

#endif  // DYNAMIC_VINO_LIB_INFERENCES_HUMAN_POSE_ESTIMATION_H