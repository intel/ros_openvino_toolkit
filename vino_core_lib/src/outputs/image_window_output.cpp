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
 * @brief a header file with declaration of ImageWindowOutput class
 * @file image_window_output.cpp
 */

#include "vino_core_lib/outputs/image_window_output.h"
#include <algorithm>
#include <string>
#include <vector>
#include "vino_core_lib/pipeline.h"

Outputs::ImageWindowOutput::ImageWindowOutput(const std::string& window_name,
                                              int focal_length)
    : window_name_(window_name), focal_length_(focal_length)
{
}

void Outputs::ImageWindowOutput::feedFrame(const cv::Mat& frame)
{
  // frame_ = frame;
  frame_ = frame.clone();
  if (camera_matrix_.empty())
  {
    int cx = frame.cols / 2;
    int cy = frame.rows / 2;
    camera_matrix_ = cv::Mat::zeros(3, 3, CV_32F);
    camera_matrix_.at<float>(0) = focal_length_;
    camera_matrix_.at<float>(2) = static_cast<float>(cx);
    camera_matrix_.at<float>(4) = focal_length_;
    camera_matrix_.at<float>(5) = static_cast<float>(cy);
    camera_matrix_.at<float>(8) = 1;
  }
}

void Outputs::ImageWindowOutput::accept(
    const std::vector<vino_core_lib::FaceDetectionResult>& results)
{
  // std::cout<<"call face"<<std::endl;
  if (outputs_.size() == 0)
  {
    initOutputs(results.size());
  }
  if (outputs_.size() != results.size())
  {
    // throw std::logic_error("size is not equal!");
    slog::err << "the size of Face Detection and Output Vector is not equal!"
              << slog::endl;
    return;
  }

  for (unsigned i = 0; i < results.size(); i++)
  {
    // outputs_[i].desc.str("");
    outputs_[i].rect = results[i].getLocation();

    auto fd_conf = results[i].getConfidence();
    if (fd_conf >= 0)
    {
      std::ostringstream ostream;
      ostream << "[" << std::fixed << std::setprecision(3) << fd_conf << "]";
      outputs_[i].desc += ostream.str();
    }
  }
}

void Outputs::ImageWindowOutput::accept(
    const std::vector<vino_core_lib::EmotionsResult>& results)
{
  if (outputs_.size() == 0)
  {
    initOutputs(results.size());
  }
  if (outputs_.size() != results.size())
  {
    // throw std::logic_error("size is not equal!");
    slog::err << "the size of Emotion Detection and Output Vector is not equal!"
              << slog::endl;
    return;
  }
  for (unsigned i = 0; i < results.size(); i++)
  {
    std::ostringstream ostream;
    ostream << "[" << results[i].getLabel() << "]";
    outputs_[i].desc += ostream.str();
  }
}

void Outputs::ImageWindowOutput::accept(
    const std::vector<vino_core_lib::AgeGenderResult>& results)
{
  if (outputs_.size() == 0)
  {
    initOutputs(results.size());
  }
  if (outputs_.size() != results.size())
  {
    // throw std::logic_error("size is not equal!");
    slog::err
        << "the size of AgeGender Detection and Output Vector is not equal!"
        << slog::endl;
    return;
  }
  for (unsigned i = 0; i < results.size(); i++)
  {
    std::ostringstream ostream;
    // auto age = results[i].getAge();
    ostream << "[Y" << std::fixed << std::setprecision(0) << results[i].getAge()
            << "]";
    outputs_[i].desc += ostream.str();

    auto male_prob = results[i].getMaleProbability();
    if (male_prob < 0.5)
    {
      outputs_[i].scalar = cv::Scalar(0, 0, 255);
    }
  }
}

cv::Point Outputs::ImageWindowOutput::calcAxis(cv::Mat r, double cx, double cy,
                                               double cz, cv::Point cp)
{
  cv::Mat Axis(3, 1, CV_32F);
  Axis.at<float>(0) = cx;
  Axis.at<float>(1) = cy;
  Axis.at<float>(2) = cz;
  cv::Mat o(3, 1, CV_32F, cv::Scalar(0));
  o.at<float>(2) = camera_matrix_.at<float>(0);
  Axis = r * Axis + o;
  cv::Point point;
  point.x = static_cast<int>(
      (Axis.at<float>(0) / Axis.at<float>(2) * camera_matrix_.at<float>(0)) +
      cp.x);
  point.y = static_cast<int>(
      (Axis.at<float>(1) / Axis.at<float>(2) * camera_matrix_.at<float>(4)) +
      cp.y);
  return point;
}

cv::Mat Outputs::ImageWindowOutput::getRotationTransform(double yaw,
                                                         double pitch,
                                                         double roll)
{
  pitch *= CV_PI / 180.0;
  yaw *= CV_PI / 180.0;
  roll *= CV_PI / 180.0;
  cv::Matx33f Rx(1, 0, 0, 0, cos(pitch), -sin(pitch), 0, sin(pitch),
                 cos(pitch));
  cv::Matx33f Ry(cos(yaw), 0, -sin(yaw), 0, 1, 0, sin(yaw), 0, cos(yaw));
  cv::Matx33f Rz(cos(roll), -sin(roll), 0, sin(roll), cos(roll), 0, 0, 0, 1);
  auto r = cv::Mat(Rz * Ry * Rx);
  return r;
}

void Outputs::ImageWindowOutput::accept(
    const std::vector<vino_core_lib::HeadPoseResult>& results)
{
  if (outputs_.size() == 0)
  {
    initOutputs(results.size());
  }
  if (outputs_.size() != results.size())
  {
    // throw std::logic_error("size is not equal!");
    slog::err
        << "the size of HeadPose Detection and Output Vector is not equal!"
        << slog::endl;
    return;
  }
  for (unsigned i = 0; i < results.size(); i++)
  {
    auto result = results[i];
    double yaw = result.getAngleY();
    double pitch = result.getAngleP();
    double roll = result.getAngleR();
    double scale = 50;
    feedFrame(frame_);
    cv::Mat r = getRotationTransform(yaw, pitch, roll);
    cv::Rect location = result.getLocation();
    auto cp = cv::Point(location.x + location.width / 2,
                        location.y + location.height / 2);
    outputs_[i].hp_cp = cp;
    outputs_[i].hp_x = calcAxis(r, scale, 0, 0, cp);
    outputs_[i].hp_y = calcAxis(r, 0, -scale, 0, cp);
    outputs_[i].hp_ze = calcAxis(r, 0, 0, -scale, cp);
    outputs_[i].hp_zs = calcAxis(r, 0, 0, scale, cp);
  }
}

void Outputs::ImageWindowOutput::accept(
    const std::vector<vino_core_lib::ObjectDetectionResult>& results)
{
  // std::cout<<"call"<<std::endl;
  if (outputs_.size() == 0)
  {
    initOutputs(results.size());
  } 
  if (outputs_.size() != results.size())
  {
    // throw std::logic_error("size is not equal!");
    slog::err << "the size of Face Detection and Output Vector is not equal!"
              << slog::endl;
    return;
  }
  for (unsigned i = 0; i < results.size(); i++) 
  {
    // outputs_[i].desc.str("");
    outputs_[i].rect = results[i].getLocation();
    auto fd_conf = results[i].getConfidence();
    if (fd_conf >= 0)
    {
      std::ostringstream ostream;
      ostream << "[" << std::fixed << std::setprecision(3) << fd_conf << "]";
      outputs_[i].desc += ostream.str();
    }
    auto label = results[i].getLabel();
    outputs_[i].desc += "[" + label + "]";
    // std::cout<<"out:" << label <<std::endl;
  }
}

void Outputs::ImageWindowOutput::mergeMask(
  const std::vector<vino_core_lib::ObjectSegmentationResult> & results)
{
  std::map<std::string, int> class_color;
  for (unsigned i = 0; i < results.size(); i++) {
    std::string class_label = results[i].getLabel();
    if (class_color.find(class_label) == class_color.end()) {
      class_color[class_label] = class_color.size();
    }
    auto & color = colors_[class_color[class_label]];
    const float alpha = 0.7f;
    const float MASK_THRESHOLD = 0.5;

    cv::Rect location = results[i].getLocation();
    cv::Mat roi_img = frame_(location);
    cv::Mat mask = results[i].getMask();
    cv::Mat colored_mask(location.height, location.width, frame_.type());

    for (int h = 0; h < mask.size().height; ++h) {
      for (int w = 0; w < mask.size().width; ++w) {
        for (int ch = 0; ch < colored_mask.channels(); ++ch) {
          colored_mask.at<cv::Vec3b>(h, w)[ch] = mask.at<float>(h, w) > MASK_THRESHOLD ?
            255 * color[ch] :
            roi_img.at<cv::Vec3b>(h, w)[ch];
        }
      }
    }
    cv::addWeighted(colored_mask, alpha, roi_img, 1.0f - alpha, 0.0f, roi_img);
  }
}

void Outputs::ImageWindowOutput::accept(
  const std::vector<vino_core_lib::ObjectSegmentationResult> & results)
{
  if (outputs_.size() == 0) {
    initOutputs(results.size());
  }
  if (outputs_.size() != results.size()) {
    slog::err << "the size of Object Segmentation and Output Vector is not equal!" << slog::endl;
    return;
  }
  for (unsigned i = 0; i < results.size(); i++) {
    outputs_[i].rect = results[i].getLocation();
    auto fd_conf = results[i].getConfidence();
    if (fd_conf >= 0) {
      std::ostringstream ostream;
      ostream << "[" << std::fixed << std::setprecision(3) << fd_conf << "]";
      outputs_[i].desc += ostream.str();
    }
    auto label = results[i].getLabel();
    outputs_[i].desc += "[" + label + "]";
  }
  mergeMask(results);
}

unsigned Outputs::ImageWindowOutput::findOutput(
  const cv::Rect & result_rect)
{
  for (unsigned i = 0; i < outputs_.size(); i++) {
    if (outputs_[i].rect == result_rect) {
      return i;
    }
  }
  OutputData output;
  output.desc = "";
  output.scalar = cv::Scalar(255, 0, 0);
  outputs_.push_back(output);
  return outputs_.size() - 1;
}

void Outputs::ImageWindowOutput::accept(
  const std::vector<vino_core_lib::PersonReidentificationResult> & results)
{
  for (unsigned i = 0; i < results.size(); i++) {
    cv::Rect result_rect = results[i].getLocation();
    unsigned target_index = findOutput(result_rect);
    outputs_[target_index].rect = result_rect;
    outputs_[target_index].desc += "[" + results[i].getPersonID() + "]";
  }
}

void Outputs::ImageWindowOutput::accept(
  const std::vector<vino_core_lib::HumanPoseResult> & results)
{
  OutputData output;
  output.scalar = cv::Scalar(255, 0, 0);
  for (unsigned i = 0; i < results.size(); i++) {
    cv::Rect result_rect = results[i].getLocation();
    auto score = std::to_string(results[i].getScore());

    output.rect = result_rect;
    output.desc = "Pose " + std::to_string(i) + " [" + score.substr(0, score.find(".") + 2) + "]";
    output.kp = results[i].getKeypoints();
    outputs_.push_back(output);
  }
}

void Outputs::ImageWindowOutput::decorateFrame()
{
  if (getPipeline()->getParameters()->isGetFps())
  {
    int fps = getPipeline()->getFPS();
    std::stringstream ss;
    ss << "FPS: " << fps;
    cv::putText(frame_, ss.str(), cv::Point2f(0, 65), cv::FONT_HERSHEY_TRIPLEX,
                0.5, cv::Scalar(255, 0, 0));
  }

  for (auto o : outputs_)
  {
    auto new_y = std::max(15, o.rect.y - 15);
    cv::putText(frame_, o.desc, cv::Point2f(o.rect.x, new_y),
                cv::FONT_HERSHEY_COMPLEX_SMALL, 0.8, o.scalar);
    cv::rectangle(frame_, o.rect, o.scalar, 1);
    cv::line(frame_, o.hp_cp, o.hp_x, cv::Scalar(0, 0, 255), 2);
    cv::line(frame_, o.hp_cp, o.hp_y, cv::Scalar(0, 255, 0), 2);
    cv::line(frame_, o.hp_zs, o.hp_ze, cv::Scalar(255, 0, 0), 2);
    cv::circle(frame_, o.hp_ze, 3, cv::Scalar(255, 0, 0), 2);

    for (auto kp : o.kp)
    {
      if (kp.x >= 0)
        cv::circle(frame_, kp, 3, cv::Scalar(255, 0, 0), 1);
    }
  }

  outputs_.clear();
}
void Outputs::ImageWindowOutput::handleOutput()
{
  cv::namedWindow(window_name_, cv::WINDOW_AUTOSIZE);
  decorateFrame();
  cv::imshow(window_name_, frame_);
  if( cv::waitKey(1) > 0)
  {
     ros::shutdown();
  }
}

void Outputs::ImageWindowOutput::initOutputs(unsigned size)
{
  outputs_.resize(size);
  for (unsigned i = 0; i < size; i++)
  {
    outputs_[i].desc = "";
    outputs_[i].scalar = cv::Scalar(255, 0, 0);
  }
}