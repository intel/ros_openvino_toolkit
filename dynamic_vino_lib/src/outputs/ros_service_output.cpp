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
 * @brief a header file with declaration of RosServiceOutput class
 * @file ros_service_output.cpp
 */

#include <vector>
#include <string>
#include <memory>
#include "dynamic_vino_lib/outputs/ros_service_output.h"
#include "cv_bridge/cv_bridge.h"
#include <object_msgs/ObjectsInBoxes.h>
// Outputs::RosServiceOutput::RosServiceOutput()


void Outputs::RosServiceOutput::setServiceResponse(
  boost::shared_ptr<object_msgs::DetectObject::Response> response)
{
  if (object_msg_ptr_ != nullptr && object_msg_ptr_->objects_vector.size() > 0) {
    object_msgs::ObjectsInBoxes objs;
    objs.objects_vector = object_msg_ptr_->objects_vector;
    response->objects.push_back(objs);
  } else if (faces_msg_ptr_ != nullptr && faces_msg_ptr_ ->objects_vector.size() > 0) {
    object_msgs::ObjectsInBoxes objs; 
    objs.objects_vector = faces_msg_ptr_->objects_vector;
    response->objects.push_back(objs);
  }
}

void Outputs::RosServiceOutput::setResponseForFace(
  boost::shared_ptr<object_msgs::DetectObject::Response> response)
{
  if (faces_msg_ptr_ != nullptr && faces_msg_ptr_->objects_vector.size() > 0) {
    object_msgs::ObjectsInBoxes objs; 
    objs.objects_vector = faces_msg_ptr_->objects_vector; 
    response->objects.push_back(objs);
  }
}

void Outputs::RosServiceOutput::setServiceResponse(
  boost::shared_ptr<people_msgs::AgeGenderSrv::Response> response)
{
  if (age_gender_msg_ptr_ != nullptr) {
    response->age_gender.objects = age_gender_msg_ptr_->objects;
  }
}

void Outputs::RosServiceOutput::setServiceResponse(
  boost::shared_ptr<people_msgs::EmotionSrv::Response> response)
{
  if (emotions_msg_ptr_ != nullptr) {
    response->emotion.emotions = emotions_msg_ptr_->emotions;
  }
}

void Outputs::RosServiceOutput::setServiceResponse(
  boost::shared_ptr<people_msgs::HeadPoseSrv::Response> response)
{
  if (headpose_msg_ptr_ != nullptr) {
    response->headpose.headposes = headpose_msg_ptr_->headposes;
  }
}

void Outputs::RosServiceOutput::setServiceResponse(
  boost::shared_ptr<people_msgs::ObjectsInMasksSrv::Response> response)
  {
    slog::info << "in ObjectsInMasks service::Response ...";
    if (segmented_object_msg_ptr_ != nullptr) {
      response->segmentation.objects_vector = segmented_object_msg_ptr_->objects_vector;
    }
  }
void Outputs::RosServiceOutput::setServiceResponse(
  boost::shared_ptr<people_msgs::ReidentificationSrv::Response> response)
  {
    slog::info << "in Reidentification service::Response ...";
    if (person_reid_msg_ptr_ != nullptr) {
      response->reidentification.reidentified_vector = person_reid_msg_ptr_->reidentified_vector;
    }
  }

void Outputs::RosServiceOutput::setServiceResponse(
  boost::shared_ptr<people_msgs::PeopleSrv::Response> response)
{
  slog::info << "in People::Response ...";
  if (faces_msg_ptr_ != nullptr) {
    slog::info << "[FACES],";
    response->persons.faces = faces_msg_ptr_->objects_vector;
  } else if (object_msg_ptr_ != nullptr) {
    slog::info << "[FACES(objects)],";
    response->persons.faces = object_msg_ptr_->objects_vector;
  }
  if (age_gender_msg_ptr_ != nullptr) {
    slog::info << "[AGE_GENDER],";
    response->persons.agegenders = age_gender_msg_ptr_->objects;
  }
  if (emotions_msg_ptr_ != nullptr) {
    slog::info << "[EMOTION],";
    response->persons.emotions = emotions_msg_ptr_->emotions;
  }
  if (headpose_msg_ptr_ != nullptr) {
    slog::info << "[HEADPOSE],";
    response->persons.headposes = headpose_msg_ptr_->headposes;
  }
  slog::info << "DONE!" << slog::endl;
}

void Outputs::RosServiceOutput::clearData()
{
  faces_msg_ptr_ = nullptr;
  object_msg_ptr_ = nullptr;
  age_gender_msg_ptr_ = nullptr;
  emotions_msg_ptr_ = nullptr;
  headpose_msg_ptr_ = nullptr;
  segmented_object_msg_ptr_ = nullptr;
  person_reid_msg_ptr_ = nullptr;
}
