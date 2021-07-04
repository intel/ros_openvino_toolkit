// TODO: Add license

/**
 * @brief A header file with declaration for FaceDetectionModel Class
 * @file human_pose_estimation_model.h
 */

#ifndef DYNAMIC_VINO_LIB_MODELS_HUMAN_POSE_ESTIMATION_MODEL_H
#define DYNAMIC_VINO_LIB_MODELS_HUMAN_POSE_ESTIMATION_MODEL_H

#include <string>
#include "dynamic_vino_lib/models/base_model.h"

namespace Models
{
/**
 * @class HumanPoseEstimationModel
 * @brief This class generates the human pose estimation model.
 */
class HumanPoseEstimationModel : public BaseModel
{
public:
  HumanPoseEstimationModel(const std::string& model_loc, int batch_size = 1);

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
  // const std::string getModelName() const override;
  const std::string getModelCategory() const override;
  bool updateLayerProperty(InferenceEngine::CNNNetReader::Ptr) override;

protected:
  // void checkLayerProperty(const InferenceEngine::CNNNetReader::Ptr&) override;
  // void setLayerProperty(InferenceEngine::CNNNetReader::Ptr) override;

 private:
  std::string input_;
  std::string output_keypoints_;
  std::string output_heatmap_;
};

}  // namespace Models

#endif  // DYNAMIC_VINO_LIB_MODELS_HUMAN_POSE_ESTIMATION_MODEL_H