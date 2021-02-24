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
 * @brief A header file with declaration for FaceReidentification Class
 * @file face_reidentification.hpp
 */
#ifndef DYNAMIC_VINO_LIB__INFERENCES__FACE_REIDENTIFICATION_H_
#define DYNAMIC_VINO_LIB__INFERENCES__FACE_REIDENTIFICATION_H_
#include "dynamic_vino_lib/engines/engine.h"
#include "dynamic_vino_lib/inferences/base_inference.h"
#include "dynamic_vino_lib/inferences/base_reidentification.h"
#include "dynamic_vino_lib/models/face_reidentification_model.h"
#include "inference_engine.hpp"
#include "opencv2/opencv.hpp"
#include <memory>
#include <string>
#include <vector>
// namespace
namespace dynamic_vino_lib
{
/**
 * @class FaceReidentificationResult
 * @brief Class for storing and processing face reidentification result.
 */
class FaceReidentificationResult : public Result
{
public:
  friend class FaceReidentification;
  explicit FaceReidentificationResult(const cv::Rect& location);
  std::string getFaceID() const
  {
    return face_id_;
  }

private:
  std::string face_id_ = "No.#";
};

/**
 * @class FaceReidentification
 * @brief Class to load face reidentification model and perform face
 * reidentification.
 */
class FaceReidentification : public BaseInference
{
public:
  using Result = dynamic_vino_lib::FaceReidentificationResult;
  explicit FaceReidentification(double);
  ~FaceReidentification() override;
  /**
   * @brief Load the face reidentification model.
   */
  void loadNetwork(std::shared_ptr<Models::FaceReidentificationModel>);
  /**
   * @brief Enqueue a frame to this class.
   * The frame will be buffered but not infered yet.
   * @param[in] frame The frame to be enqueued.
   * @param[in] input_frame_loc The location of the enqueued frame with respect
   * to the frame generated by the input device.
   * @return Whether this operation is successful.
   */
  bool enqueue(const cv::Mat&, const cv::Rect&) override;
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
   * @brief Show the observed reidentification result either through image
   window
     or ROS topic.
   */
  void observeOutput(const std::shared_ptr<Outputs::BaseOutput>& output);
  /**
   * @brief Get the name of the Inference instance.
   * @return The name of the Inference instance.
   */
  const std::string getName() const override;
  const std::vector<cv::Rect> getFilteredROIs(const std::string filter_conditions) const override;

private:
  std::shared_ptr<Models::FaceReidentificationModel> valid_model_;
  std::vector<Result> results_;
  std::shared_ptr<dynamic_vino_lib::Tracker> face_tracker_;
};
}  // namespace dynamic_vino_lib
#endif  // DYNAMIC_VINO_LIB__INFERENCES__FACE_REIDENTIFICATION_HPP_
