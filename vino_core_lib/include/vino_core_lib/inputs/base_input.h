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
 * @brief A header file with declaration for BaseInput Class
 * @file base_input.h
 */
#ifndef VINO_CORE_LIB__INPUTS__BASE_INPUT_H
#define VINO_CORE_LIB__INPUTS__BASE_INPUT_H

#include <opencv2/opencv.hpp>
#include "vino_core_lib/vino_factory.h"
#include "vino_core_lib/inputs/ros_handler.h"

/**
 * @class BaseInputDevice
 * @brief This class is an interface for three kinds of
 * input devices: realsense camera, standard camera and video
 */
namespace Input
{
struct Config
{
  std::string path;
};

class BaseInputDevice : public RosHandler
{
public:
  BaseInputDevice() = default;
  virtual ~BaseInputDevice() = default;
  /**
   * @brief Initialize the input device,
   * @return Whether the input device is successfully setup.
   */
  virtual bool init(const std::string &str) = 0;
  /**
   * @brief Initialize the input device,
   * for cameras, it will turn the camera on and get ready to read frames,
   * for videos, it will open a video file.
   * @return Whether the input device is successfully turned on.
   */
  virtual bool initialize() = 0;
  /**
   * @brief Initialize the input device with given width and height.
   * @return Whether the input device is successfully turned on.
   */
  virtual bool initialize(size_t width, size_t height) = 0;
  /**
   * @brief Read next frame, and give the value to argument frame.
   * @return Whether the next frame is successfully read.
   */
  virtual bool read(cv::Mat* frame) = 0;
  virtual bool readService(cv::Mat* frame, std::string config_path)
  {
    return true;
  }
  virtual void config(const Config&)
  {
  }
  /**
   * @brief Get the width of the frame read from input device.
   * @return The width of the frame read from input device.
   */
  inline size_t getWidth()
  {
    return width_;
  }
  /**
   * @brief Set the width of the frame read from input device.
   * @param[in] width Width to be set for the frame.
   */
  inline void setWidth(size_t width)
  {
    width_ = width;
  }
  /**
   * @brief Get the height of the frame read from input device.
   * @return The height of the frame read from input device.
   */
  inline size_t getHeight()
  {
    return height_;
  }
  /**
   * @brief Set the height of the frame read from input device.
   * @param[in] width Width to be set for the frame.
   */
  inline void setHeight(size_t height)
  {
    height_ = height;
  }
  /**
   * @brief Check whether the input device is successfully initiated.
   * @return Whether the input device is successfully initiated.
   */
  inline bool isInit()
  {
    return is_init_;
  }
  /**
   * @brief Set the initialization state for input device.
   * @param[in] is_init The initialization state to be set.
   */
  inline void setInitStatus(bool is_init)
  {
    is_init_ = is_init;
  }

private:
  size_t width_ = 0;   // 0 means using the original size
  size_t height_ = 0;  // 0 means using the original size
  bool is_init_ = false;
};
}  // namespace Input

#define REG_INPUT_FACTORY VinoFactory<std::string, Input::BaseInputDevice>
#define REG_INPUT(BASE, key)  static REG_INPUT_FACTORY::TReg<Input::BASE> gs_input##_(key)

#endif  // VINO_CORE_LIB__INPUTS__BASE_INPUT_H
