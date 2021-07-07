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

/**
 * @brief a header file with declaration of Pipeline class
 * @file pipeline_param.h
 */
#ifndef DYNAMIC_VINO_LIB_PIPELINE_PARAMS_H
#define DYNAMIC_VINO_LIB_PIPELINE_PARAMS_H

#include <atomic>
#include <future>
#include <map>
#include <memory>
#include <mutex>
#include <set>
#include <string>

#include "dynamic_vino_lib/inferences/base_inference.h"
#include "dynamic_vino_lib/inputs/standard_camera.h"
#include "dynamic_vino_lib/outputs/base_output.h"
#include "opencv2/opencv.hpp"
#include "vino_param_lib/param_manager.h"

const char kInputType_Image[] = "Image";
const char kInputType_Video[] = "Video";
const char kInputType_StandardCamera[] = "StandardCamera";
const char kInputType_IpCamera[] = "IpCamera";
const char kInputType_CameraTopic[] = "RealSenseCameraTopic";
const char kInputType_ImageTopic[] = "ImageTopic";
const char kInputType_RealSenseCamera[] = "RealSenseCamera";
const char kInputType_ServiceImage[] = "ServiceImage";

const char kOutputTpye_RViz[] = "RViz";
const char kOutputTpye_ImageWindow[] = "ImageWindow";
const char kOutputTpye_RosTopic[] = "RosTopic";
const char kOutputTpye_RosService[] = "RosService";

const char kInferTpye_FaceDetection[] = "FaceDetection";
const char kInferTpye_AgeGenderRecognition[] = "AgeGenderRecognition";
const char kInferTpye_EmotionRecognition[] = "EmotionRecognition";
const char kInferTpye_HeadPoseEstimation[] = "HeadPoseEstimation";
const char kInferTpye_HumanPoseEstimation[] = "HumanPoseEstimation";
const char kInferTpye_ObjectDetection[] = "ObjectDetection";
const char kInferTpye_ObjectSegmentation[] = "ObjectSegmentation";
const char kInferTpye_PersonReidentification[] = "PersonReidentification";
const char kInferTpye_ObjectDetectionTypeSSD[] = "SSD";
const char kInferTpye_ObjectDetectionTypeYolov2voc[] = "yolov2-voc";
const char kInferTpye_LandmarksDetection[] = "LandmarksDetection";
const char kInferTpye_FaceReidentification[] = "FaceReidentification";
const char kInferTpye_PersonAttribsDetection[] = "PersonAttribsDetection";
const char kInferTpye_VehicleAttribsDetection[] = "VehicleAttribsDetection";
const char kInferTpye_LicensePlateDetection[] = "LicensePlateDetection";
/**
 * @class PipelineParams
 * @brief This class is a pipeline parameter management that stores parameters
 * of a given pipeline
 */
class PipelineParams
{
public:
  explicit PipelineParams(const std::string& name);
  explicit PipelineParams(const Params::ParamManager::PipelineRawData& params);
  Params::ParamManager::PipelineRawData getPipeline(const std::string& name);
  PipelineParams& operator=(const Params::ParamManager::PipelineRawData& params);
  void update();
  void update(const Params::ParamManager::PipelineRawData& params);
  bool isOutputTo(std::string& name);
  bool isGetFps();
  std::string findFilterConditions(const std::string& input, const std::string& output);

  const std::string kInputType_Image = "Image";
  const std::string kOutputTpye_RViz = "RViz";

private:
  Params::ParamManager::PipelineRawData params_;
};

#endif  // DYNAMIC_VINO_LIB_PIPELINE_PARAMS_H
