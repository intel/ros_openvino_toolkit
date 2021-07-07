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

#include "dynamic_vino_lib/outputs/image_window_output.h"
#include <algorithm>
#include <string>
#include <vector>
#include <map>
#include <iomanip>

#include "dynamic_vino_lib/outputs/image_window_output.h"
#include "dynamic_vino_lib/pipeline.h"

Outputs::ImageWindowOutput::ImageWindowOutput(const std::string& output_name, int focal_length)
  : BaseOutput(output_name), focal_length_(focal_length)
{
  cv::namedWindow(output_name_, cv::WINDOW_AUTOSIZE);
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

unsigned Outputs::ImageWindowOutput::findOutput(const cv::Rect& result_rect)
{
  for (unsigned i = 0; i < outputs_.size(); i++)
  {
    if (outputs_[i].rect == result_rect)
    {
      return i;
    }
  }
  OutputData output;
  output.desc = "";
  output.scalar = cv::Scalar(255, 0, 0);
  outputs_.push_back(output);
  return outputs_.size() - 1;
}

void Outputs::ImageWindowOutput::accept(const std::vector<dynamic_vino_lib::LicensePlateDetectionResult>& results)
{
  for (unsigned i = 0; i < results.size(); i++)
  {
    cv::Rect result_rect = results[i].getLocation();
    unsigned target_index = findOutput(result_rect);
    outputs_[target_index].rect = result_rect;
    outputs_[target_index].desc += ("[" + results[i].getLicense() + "]");
  }
}

void Outputs::ImageWindowOutput::accept(const std::vector<dynamic_vino_lib::VehicleAttribsDetectionResult>& results)
{
  for (unsigned i = 0; i < results.size(); i++)
  {
    cv::Rect result_rect = results[i].getLocation();
    unsigned target_index = findOutput(result_rect);
    outputs_[target_index].rect = result_rect;
    outputs_[target_index].desc += ("[" + results[i].getColor() + "," + results[i].getType() + "]");
  }
}

void Outputs::ImageWindowOutput::accept(const std::vector<dynamic_vino_lib::FaceDetectionResult>& results)
{
  for (unsigned i = 0; i < results.size(); i++)
  {
    cv::Rect result_rect = results[i].getLocation();
    unsigned target_index = findOutput(result_rect);
    outputs_[target_index].rect = result_rect;
    auto fd_conf = results[i].getConfidence();
    if (fd_conf >= 0)
    {
      std::ostringstream ostream;
      ostream << "[" << std::fixed << std::setprecision(3) << fd_conf << "]";
      outputs_[target_index].desc += ostream.str();
    }
  }
}

void Outputs::ImageWindowOutput::accept(const std::vector<dynamic_vino_lib::EmotionsResult>& results)
{
  for (unsigned i = 0; i < results.size(); i++)
  {
    cv::Rect result_rect = results[i].getLocation();
    unsigned target_index = findOutput(result_rect);
    outputs_[target_index].rect = result_rect;
    std::ostringstream ostream;
    ostream << "[" << results[i].getLabel() << "]";
    outputs_[target_index].desc += ostream.str();
  }
}

void Outputs::ImageWindowOutput::accept(const std::vector<dynamic_vino_lib::AgeGenderResult>& results)
{
  for (unsigned i = 0; i < results.size(); i++)
  {
    cv::Rect result_rect = results[i].getLocation();
    unsigned target_index = findOutput(result_rect);
    outputs_[target_index].rect = result_rect;
    std::ostringstream ostream;
    ostream << "[Y" << std::fixed << std::setprecision(0) << results[i].getAge() << "]";
    outputs_[target_index].desc += ostream.str();

    auto male_prob = results[i].getMaleProbability();
    if (male_prob < 0.5)
    {
      outputs_[target_index].scalar = cv::Scalar(0, 0, 255);
    }
  }
}

cv::Point Outputs::ImageWindowOutput::calcAxis(cv::Mat r, double cx, double cy, double cz, cv::Point cp)
{
  cv::Mat Axis(3, 1, CV_32F);
  Axis.at<float>(0) = cx;
  Axis.at<float>(1) = cy;
  Axis.at<float>(2) = cz;
  cv::Mat o(3, 1, CV_32F, cv::Scalar(0));
  o.at<float>(2) = camera_matrix_.at<float>(0);
  Axis = r * Axis + o;
  cv::Point point;
  point.x = static_cast<int>((Axis.at<float>(0) / Axis.at<float>(2) * camera_matrix_.at<float>(0)) + cp.x);
  point.y = static_cast<int>((Axis.at<float>(1) / Axis.at<float>(2) * camera_matrix_.at<float>(4)) + cp.y);
  return point;
}

cv::Mat Outputs::ImageWindowOutput::getRotationTransform(double yaw, double pitch, double roll)
{
  pitch *= CV_PI / 180.0;
  yaw *= CV_PI / 180.0;
  roll *= CV_PI / 180.0;
  cv::Matx33f Rx(1, 0, 0, 0, cos(pitch), -sin(pitch), 0, sin(pitch), cos(pitch));
  cv::Matx33f Ry(cos(yaw), 0, -sin(yaw), 0, 1, 0, sin(yaw), 0, cos(yaw));
  cv::Matx33f Rz(cos(roll), -sin(roll), 0, sin(roll), cos(roll), 0, 0, 0, 1);
  auto r = cv::Mat(Rz * Ry * Rx);
  return r;
}

void Outputs::ImageWindowOutput::accept(const std::vector<dynamic_vino_lib::HeadPoseResult>& results)
{
  for (unsigned i = 0; i < results.size(); i++)
  {
    auto result = results[i];
    cv::Rect result_rect = result.getLocation();
    unsigned target_index = findOutput(result_rect);
    outputs_[target_index].rect = result_rect;
    double yaw = result.getAngleY();
    double pitch = result.getAngleP();
    double roll = result.getAngleR();
    double scale = 50;
    feedFrame(frame_);
    cv::Mat r = getRotationTransform(yaw, pitch, roll);
    cv::Rect location = result.getLocation();
    auto cp = cv::Point(location.x + location.width / 2, location.y + location.height / 2);
    outputs_[target_index].hp_cp = cp;
    outputs_[target_index].hp_x = calcAxis(r, scale, 0, 0, cp);
    outputs_[target_index].hp_y = calcAxis(r, 0, -scale, 0, cp);
    outputs_[target_index].hp_ze = calcAxis(r, 0, 0, -scale, cp);
    outputs_[target_index].hp_zs = calcAxis(r, 0, 0, scale, cp);
  }
}

void Outputs::ImageWindowOutput::accept(const std::vector<dynamic_vino_lib::HumanPoseResult>& results)
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

void Outputs::ImageWindowOutput::accept(const std::vector<dynamic_vino_lib::ObjectDetectionResult>& results)
{
  for (unsigned i = 0; i < results.size(); i++)
  {
    cv::Rect result_rect = results[i].getLocation();
    unsigned target_index = findOutput(result_rect);
    outputs_[target_index].rect = result_rect;
    auto fd_conf = results[i].getConfidence();
    if (fd_conf >= 0)
    {
      std::ostringstream ostream;
      ostream << "[" << std::fixed << std::setprecision(3) << fd_conf << "]";
      outputs_[target_index].desc += ostream.str();
    }
    auto label = results[i].getLabel();
    outputs_[target_index].desc += "[" + label + "]";
  }
}


void Outputs::ImageWindowOutput::mergeMask(const std::vector<dynamic_vino_lib::ObjectSegmentationResult>& results)
{
  const float alpha = 0.5f;
  cv::Mat roi_img = frame_;
  cv::Mat colored_mask = results[0].getMask();
  cv::resize(colored_mask, colored_mask, cv::Size(frame_.size().width, frame_.size().height));
  cv::addWeighted(colored_mask, alpha, roi_img, 1.0f - alpha, 0.0f, roi_img);
}

void Outputs::ImageWindowOutput::accept(const std::vector<dynamic_vino_lib::ObjectSegmentationResult>& results)
{
  if (outputs_.size() == 0)
  {
    initOutputs(results.size());
  }
  if (outputs_.size() != results.size())
  {
    slog::err << "the size of Object Segmentation and Output Vector is not equal!" << slog::endl;
    return;
  }
  mergeMask(results);
}

void Outputs::ImageWindowOutput::accept(const std::vector<dynamic_vino_lib::PersonAttribsDetectionResult>& results)
{
  for (unsigned i = 0; i < results.size(); i++)
  {
    cv::Rect result_rect = results[i].getLocation();
    unsigned target_index = findOutput(result_rect);
    if (results[i].getMaleProbability() < 0.5)
    {
      outputs_[target_index].scalar = cv::Scalar(0, 0, 255);
    }
    else
    {
      outputs_[target_index].scalar = cv::Scalar(0, 255, 0);
    }
    outputs_[target_index].pa_top.x = results[i].getTopLocation().x * result_rect.width + result_rect.x;
    outputs_[target_index].pa_top.y = results[i].getTopLocation().y * result_rect.height + result_rect.y;
    outputs_[target_index].pa_bottom.x = results[i].getBottomLocation().x * result_rect.width + result_rect.x;
    outputs_[target_index].pa_bottom.y = results[i].getBottomLocation().y * result_rect.height + result_rect.y;

    outputs_[target_index].rect = result_rect;
    outputs_[target_index].desc += "[" + results[i].getAttributes() + "]";
  }
}

void Outputs::ImageWindowOutput::accept(const std::vector<dynamic_vino_lib::PersonReidentificationResult>& results)
{
  for (unsigned i = 0; i < results.size(); i++)
  {
    cv::Rect result_rect = results[i].getLocation();
    unsigned target_index = findOutput(result_rect);
    outputs_[target_index].rect = result_rect;
    outputs_[target_index].desc += "[" + results[i].getPersonID() + "]";
  }
}

void Outputs::ImageWindowOutput::accept(const std::vector<dynamic_vino_lib::FaceReidentificationResult>& results)
{
  for (unsigned i = 0; i < results.size(); i++)
  {
    cv::Rect result_rect = results[i].getLocation();
    unsigned target_index = findOutput(result_rect);
    outputs_[target_index].rect = result_rect;
    outputs_[target_index].desc += "[" + results[i].getFaceID() + "]";
  }
}

void Outputs::ImageWindowOutput::accept(const std::vector<dynamic_vino_lib::LandmarksDetectionResult>& results)
{
  for (unsigned i = 0; i < results.size(); i++)
  {
    cv::Rect result_rect = results[i].getLocation();
    unsigned target_index = findOutput(result_rect);
    std::vector<cv::Point> landmark_points = results[i].getLandmarks();
    for (int j = 0; j < landmark_points.size(); j++)
    {
      outputs_[target_index].landmarks.push_back(landmark_points[j]);
    }
  }
}

void Outputs::ImageWindowOutput::decorateFrame()
{
  if (getPipeline()->getParameters()->isGetFps())
  {
    int fps = getPipeline()->getFPS();
    std::stringstream ss;
    ss << "FPS: " << fps;
    cv::putText(frame_, ss.str(), cv::Point2f(0, 65), cv::FONT_HERSHEY_TRIPLEX, 0.5, cv::Scalar(255, 0, 0));
  }

  for (auto o : outputs_)
  {
    auto new_y = std::max(15, o.rect.y - 15);
    cv::putText(frame_, o.desc, cv::Point2f(o.rect.x, new_y), cv::FONT_HERSHEY_COMPLEX_SMALL, 0.8, o.scalar);
    cv::rectangle(frame_, o.rect, o.scalar, 1);
    if (o.hp_cp != o.hp_x)
    {
      cv::line(frame_, o.hp_cp, o.hp_x, cv::Scalar(0, 0, 255), 2);
    }
    if (o.hp_cp != o.hp_y)
    {
      cv::line(frame_, o.hp_cp, o.hp_y, cv::Scalar(0, 255, 0), 2);
    }
    if (o.hp_zs != o.hp_ze)
    {
      cv::line(frame_, o.hp_zs, o.hp_ze, cv::Scalar(255, 0, 0), 2);
      cv::circle(frame_, o.hp_ze, 3, cv::Scalar(255, 0, 0), 2);
    }
    for (auto kp : o.kp)
    {
      if (kp.x >= 0)
      {
        std::string score = std::to_string(kp.score).substr(0, 3);
        cv::circle(frame_, kp, 3, cv::Scalar(255, 0, 0), cv::FILLED);
        cv::putText(frame_, score, kp + cv::Point2f(5, 0),
                cv::FONT_HERSHEY_COMPLEX_SMALL, 0.5, o.scalar);
      }
    }
    const cv::Point2f absentKeypoint(-1.0f, -1.0f);
    const int stickWidth = 4;
    static const cv::Scalar colors[18] = {
        cv::Scalar(255, 0, 0),      cv::Scalar(255, 85, 0),     cv::Scalar(255, 170, 0),
        cv::Scalar(255, 255, 0),    cv::Scalar(170, 255, 0),    cv::Scalar(85, 255, 0),
        cv::Scalar(0, 255, 0),      cv::Scalar(0, 255, 85),     cv::Scalar(0, 255, 170),
        cv::Scalar(0, 255, 255),    cv::Scalar(0, 170, 255),    cv::Scalar(0, 85, 255),
        cv::Scalar(0, 0, 255),      cv::Scalar(85, 0, 255),     cv::Scalar(170, 0, 255),
        cv::Scalar(255, 0, 255),    cv::Scalar(255, 0, 170),    cv::Scalar(255, 0, 85)
    };
    static const std::pair<int, int> limbKeypointsIds[] = {
        {1, 2},  {1, 5},   {2, 3},
        {3, 4},  {5, 6},   {6, 7},
        {1, 8},  {8, 9},   {9, 10},
        {1, 11}, {11, 12}, {12, 13},
        {1, 0},  {0, 14},  {14, 16},
        {0, 15}, {15, 17}
    };
    for (const auto& limbKeypointsId : limbKeypointsIds) 
    {
        std::pair<cv::Point2f, cv::Point2f> limbKeypoints(o.kp[limbKeypointsId.first], 
            o.kp[limbKeypointsId.second]);
        if (limbKeypoints.first == absentKeypoint || limbKeypoints.second == absentKeypoint) 
        {
            continue;
        }

        float meanX = (limbKeypoints.first.x + limbKeypoints.second.x) / 2;
        float meanY = (limbKeypoints.first.y + limbKeypoints.second.y) / 2;
        cv::Point difference = limbKeypoints.first - limbKeypoints.second;
        double length = std::sqrt(difference.x * difference.x + difference.y * difference.y);
        int angle = static_cast<int>(std::atan2(difference.y, difference.x) * 180 / CV_PI);
        std::vector<cv::Point> polygon;
        cv::ellipse2Poly(cv::Point2d(meanX, meanY), cv::Size2d(length / 2, stickWidth),
                          angle, 0, 360, 1, polygon);
        cv::fillConvexPoly(frame_, polygon, colors[limbKeypointsId.second]);
    }
    //TODO Corsair-cxs
    auto faceBoundingBoxWidth = o.rect.width;
    int lmRadius = static_cast<int>(0.01 * faceBoundingBoxWidth + 1);
    for (unsigned long i = 0; i < o.landmarks.size(); i++)
    {
      cv::circle(frame_, o.landmarks[i], lmRadius, cv::Scalar(0, 255, 255), -1);
    }
  }

  outputs_.clear();
}
void Outputs::ImageWindowOutput::handleOutput()
{
  if (frame_.cols == 0 || frame_.rows == 0)
  {
    return;
  }
  decorateFrame();
  cv::imshow(output_name_, frame_);
  cv::waitKey(1);
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