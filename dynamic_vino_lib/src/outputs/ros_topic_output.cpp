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
 * @brief a header file with declaration of RosTopicOutput class
 * @file ros_topic_output.cpp
 */

#include "dynamic_vino_lib/outputs/ros_topic_output.h"
#include <chrono>
#include <memory>
#include <string>
#include <vector>

Outputs::RosTopicOutput::RosTopicOutput()
{
  pub_face_ =
      nh_.advertise<object_msgs::ObjectsInBoxes>("/openvino_toolkit/faces", 16);
  pub_emotion_ = nh_.advertise<people_msgs::EmotionsStamped>(
      "/openvino_toolkit/emotions", 16);
  pub_age_gender_ = nh_.advertise<people_msgs::AgeGenderStamped>(
      "/openvino_toolkit/age_genders", 16);
  pub_headpose_ = nh_.advertise<people_msgs::HeadPoseStamped>(
      "/openvino_toolkit/headposes", 16);
  pub_object_ = nh_.advertise<object_msgs::ObjectsInBoxes>(
      "/openvino_toolkit/objects", 16);

  emotions_msg_ptr_ = NULL;
  faces_msg_ptr_ = NULL;
  age_gender_msg_ptr_ = NULL;
  headpose_msg_ptr_ = NULL;
  object_msg_ptr_ = NULL;
}

void Outputs::RosTopicOutput::feedFrame(const cv::Mat& frame) {}

void Outputs::RosTopicOutput::accept(
    const std::vector<dynamic_vino_lib::FaceDetectionResult>& results)
{
  faces_msg_ptr_ = std::make_shared<object_msgs::ObjectsInBoxes>();

  object_msgs::ObjectInBox face;
  for (auto r : results)
  {
    auto loc = r.getLocation();
    face.roi.x_offset = loc.x;
    face.roi.y_offset = loc.y;
    face.roi.width = loc.width;
    face.roi.height = loc.height;
    face.object.object_name = r.getLabel();
    face.object.probability = r.getConfidence();
    faces_msg_ptr_->objects_vector.push_back(face);
  }
}

void Outputs::RosTopicOutput::accept(
    const std::vector<dynamic_vino_lib::EmotionsResult>& results)
{
  emotions_msg_ptr_ = std::make_shared<people_msgs::EmotionsStamped>();

  people_msgs::Emotion emotion;
  for (auto r : results)
  {
    auto loc = r.getLocation();
    emotion.roi.x_offset = loc.x;
    emotion.roi.y_offset = loc.y;
    emotion.roi.width = loc.width;
    emotion.roi.height = loc.height;
    emotion.emotion = r.getLabel();
    emotions_msg_ptr_->emotions.push_back(emotion);
  }
}

void Outputs::RosTopicOutput::accept(
    const std::vector<dynamic_vino_lib::AgeGenderResult>& results)
{
  age_gender_msg_ptr_ = std::make_shared<people_msgs::AgeGenderStamped>();

  people_msgs::AgeGender ag;
  for (auto r : results)
  {
    auto loc = r.getLocation();
    ag.roi.x_offset = loc.x;
    ag.roi.y_offset = loc.y;
    ag.roi.width = loc.width;
    ag.roi.height = loc.height;
    ag.age = r.getAge();
    auto male_prob = r.getMaleProbability();
    if (male_prob > 0.5)
    {
      ag.gender = "Male";
      ag.gender_confidence = male_prob;
    }
    else
    {
      ag.gender = "Female";
      ag.gender_confidence = 1.0 - male_prob;
    }
    age_gender_msg_ptr_->objects.push_back(ag);
  }
}

void Outputs::RosTopicOutput::accept(
    const std::vector<dynamic_vino_lib::HeadPoseResult>& results)
{
  headpose_msg_ptr_ = std::make_shared<people_msgs::HeadPoseStamped>();

  people_msgs::HeadPose hp;
  for (auto r : results)
  {
    auto loc = r.getLocation();
    hp.roi.x_offset = loc.x;
    hp.roi.y_offset = loc.y;
    hp.roi.width = loc.width;
    hp.roi.height = loc.height;
    hp.yaw = r.getAngleY();
    hp.pitch = r.getAngleP();
    hp.roll = r.getAngleR();
    headpose_msg_ptr_->headposes.push_back(hp);
  }
}

void Outputs::RosTopicOutput::accept(
    const std::vector<dynamic_vino_lib::ObjectDetectionResult>& results)
{
  object_msg_ptr_ = std::make_shared<object_msgs::ObjectsInBoxes>();

  object_msgs::ObjectInBox hp;
  for (auto r : results)
  {
    auto loc = r.getLocation();
    hp.roi.x_offset = loc.x;
    hp.roi.y_offset = loc.y;
    hp.roi.width = loc.width;
    hp.roi.height = loc.height;
    hp.object.object_name = r.getLabel();
    hp.object.probability = r.getConfidence();
    object_msg_ptr_->objects_vector.push_back(hp);
  }
}

void Outputs::RosTopicOutput::handleOutput()
{
  std_msgs::Header header = getHeader();
  if (faces_msg_ptr_ != NULL)
  {
    object_msgs::ObjectsInBoxes faces_msg;
    faces_msg.header = header;
    faces_msg.objects_vector.swap(faces_msg_ptr_->objects_vector);

    pub_face_.publish(faces_msg);
    faces_msg_ptr_ = nullptr;
  }
  if (emotions_msg_ptr_ != nullptr)
  {
    people_msgs::EmotionsStamped emotions_msg;
    emotions_msg.header = header;
    emotions_msg.emotions.swap(emotions_msg_ptr_->emotions);

    pub_emotion_.publish(emotions_msg);
    emotions_msg_ptr_ = nullptr;
  }
  if (age_gender_msg_ptr_ != nullptr)
  {
    people_msgs::AgeGenderStamped age_gender_msg;
    age_gender_msg.header = header;
    age_gender_msg.objects.swap(age_gender_msg_ptr_->objects);

    pub_age_gender_.publish(age_gender_msg);
    age_gender_msg_ptr_ = nullptr;
  }
  if (headpose_msg_ptr_ != nullptr)
  {
    people_msgs::HeadPoseStamped headpose_msg;
    headpose_msg.header = header;
    headpose_msg.headposes.swap(headpose_msg_ptr_->headposes);

    pub_headpose_.publish(headpose_msg);
    headpose_msg_ptr_ = nullptr;
  }
  if (object_msg_ptr_ != nullptr)
  {
    object_msgs::ObjectsInBoxes object_msg;
    object_msg.header = header;
    object_msg.objects_vector.swap(object_msg_ptr_->objects_vector);

    pub_object_.publish(object_msg);
    object_msg_ptr_ = nullptr;
  }
}

std_msgs::Header Outputs::RosTopicOutput::getHeader()
{
  std_msgs::Header header;
  header.frame_id = "default_camera";

  std::chrono::high_resolution_clock::time_point tp =
      std::chrono::high_resolution_clock::now();
  int64 ns = tp.time_since_epoch().count();
  header.stamp.sec = ns / 1000000000;
  header.stamp.nsec = ns % 1000000000;
  return header;
}
