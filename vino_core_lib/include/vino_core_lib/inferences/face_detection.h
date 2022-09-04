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
 * @brief A header file with declaration for FaceDetection Class
 * @file face_detection.h
 */
#ifndef VINO_CORE_LIB__INFERENCES__FACE_DETECTION_H
#define VINO_CORE_LIB__INFERENCES__FACE_DETECTION_H

#include <object_msgs/Object.h>
#include <object_msgs/ObjectInBox.h>
#include <object_msgs/ObjectsInBoxes.h>
#include <ros/ros.h>

#include <memory>
#include <string>
#include <vector>

#include "vino_core_lib/engines/engine.h"
#include "vino_core_lib/inferences/base_inference.h"
#include "vino_core_lib/inferences/object_detection.h"
#include "vino_core_lib/models/face_detection_model.h"
#include "inference_engine.hpp"
#include "opencv2/opencv.hpp"

// namespace
namespace vino_core_lib
{
/**
 * @class FaceDetectionResult
 * @brief Class for storing and processing face detection result.
 */
class FaceDetectionResult : public ObjectDetectionResult
{
public:
  explicit FaceDetectionResult(const cv::Rect& location);
};

/**
 * @class FaceDetection
 * @brief Class to load face detection model and perform face detection.
 */
class FaceDetection : public ObjectDetection
{
public:
  explicit FaceDetection(bool, double);
  FaceDetection(){};
  ~FaceDetection() override {};
};
}  // namespace vino_core_lib
#endif  // VINO_CORE_LIB__INFERENCES__FACE_DETECTION_H
