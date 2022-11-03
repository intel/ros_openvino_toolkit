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
 * @brief A header file with declaration for Video class
 * @file video_input.h
 */
#ifndef VINO_CORE_LIB__INPUTS__VIDEO_INPUT_H
#define VINO_CORE_LIB__INPUTS__VIDEO_INPUT_H

#include <opencv2/opencv.hpp>
#include <string>
#include "vino_core_lib/inputs/base_input.h"

namespace Input
{
/**
 * @class Video
 * @brief Class for recieving a video file as input.
 */
class Video : public BaseInputDevice
{
public:
  Video() {};
  Video(const std::string& video) : video_(video) {};
  /**
   * @brief Read a video file from the file path.
   * @param[in] An video file path.
   * @return Whether the input device is successfully setup.
   */
  bool init(const std::string &video) override {video_.assign(video); initialize();};
  /**
   * @brief Read a video file from the file path.
   * @param[in] An video file path.
   * @return Whether the input device is successfully turned on.
   */
  bool initialize() override;
  /**
   * @brief Initialize the input device with given width and height.
   * No implementation for Video class.
   * @return Whether the input device is successfully turned on.
   */
  bool initialize(size_t width, size_t height) override;
  /**
   * @brief Read next frame, and give the value to argument frame.
   * @return Whether the next frame is successfully read.
   */
  bool read(cv::Mat* frame) override;

private:
  cv::VideoCapture cap;
  std::string video_;
};
}  // namespace Input

#endif  // VINO_CORE_LIB__INPUTS__VIDEO_INPUT_H
