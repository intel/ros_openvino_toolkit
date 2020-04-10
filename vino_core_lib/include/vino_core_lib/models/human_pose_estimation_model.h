// TODO: Add license

/**
 * @brief A header file with declaration for FaceDetectionModel Class
 * @file human_pose_estimation_model.h
 */

#ifndef VINO_CORE_LIB_MODELS_HUMAN_POSE_ESTIMATION_MODEL_H
#define VINO_CORE_LIB_MODELS_HUMAN_POSE_ESTIMATION_MODEL_H

#include <string>
#include "vino_core_lib/models/base_model.h"

namespace Models
{
/**
 * @class HumanPoseEstimationModel
 * @brief This class generates the human pose estimation model.
 */
class HumanPoseEstimationModel : public BaseModel
{
public:
    HumanPoseEstimationModel(const std::string&, int, int, int);

  /**
   * @brief Get the input name.
   * @return Input name.
   */
  inline const std::string getInputName() const
  {
    return input_;
  }
  /**
   * @brief Get the age from the detection reuslt.
   * @return Detected age.
   */
  inline const std::string getOutputKeypointsName() const
  {
    return output_keypoints_;
  }
  /**
   * @brief Get the gender from the detection reuslt.
   * @return Detected gender.
   */
  inline const std::string getOutputHeatmapName() const
  {
    return output_heatmap_;
  }
  /**
   * @brief Get the name of this detection model.
   * @return Name of the model.
   */
  const std::string getModelName() const override;

protected:
  void checkLayerProperty(const InferenceEngine::CNNNetReader::Ptr&) override;
  void setLayerProperty(InferenceEngine::CNNNetReader::Ptr) override;

 private:
  std::string input_;
  std::string output_keypoints_;
  std::string output_heatmap_;
};

}  // namespace Models

#endif  // VINO_CORE_LIB_MODELS_HUMAN_POSE_ESTIMATION_MODEL_H