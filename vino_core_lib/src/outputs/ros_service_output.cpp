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
#include "vino_core_lib/outputs/ros_service_output.h"
#include "cv_bridge/cv_bridge.h"
#include <object_msgs/ObjectsInBoxes.h>

void Outputs::RosServiceOutput::setServiceResponse(boost::shared_ptr<object_msgs::DetectObject::Response> response)
{
  if (detected_objects_topic_ != nullptr && detected_objects_topic_->objects_vector.size() > 0)
  {
    object_msgs::ObjectsInBoxes objs;
    objs.objects_vector = detected_objects_topic_->objects_vector;
    response->objects.push_back(objs);
  }
  else if (faces_topic_ != nullptr && faces_topic_->objects_vector.size() > 0)
  {
    object_msgs::ObjectsInBoxes objs;
    objs.objects_vector = faces_topic_->objects_vector;
    response->objects.push_back(objs);
  }
}

void Outputs::RosServiceOutput::setResponseForFace(boost::shared_ptr<object_msgs::DetectObject::Response> response)
{
  if (faces_topic_ != nullptr && faces_topic_->objects_vector.size() > 0)
  {
    object_msgs::ObjectsInBoxes objs;
    objs.objects_vector = faces_topic_->objects_vector;
    response->objects.push_back(objs);
  }
}

void Outputs::RosServiceOutput::setServiceResponse(boost::shared_ptr<vino_people_msgs::AgeGenderSrv::Response> response)
{
  if (age_gender_topic_ != nullptr)
  {
    response->age_gender.objects = age_gender_topic_->objects;
  }
}

void Outputs::RosServiceOutput::setServiceResponse(boost::shared_ptr<vino_people_msgs::EmotionSrv::Response> response)
{
  if (emotions_topic_ != nullptr)
  {
    response->emotion.emotions = emotions_topic_->emotions;
  }
}

void Outputs::RosServiceOutput::setServiceResponse(boost::shared_ptr<vino_people_msgs::HeadPoseSrv::Response> response)
{
  if (headpose_topic_ != nullptr)
  {
    response->headpose.headposes = headpose_topic_->headposes;
  }
}

void Outputs::RosServiceOutput::setServiceResponse(boost::shared_ptr<vino_people_msgs::ObjectsInMasksSrv::Response> response)
{
  slog::info << "in ObjectsInMasks service::Response ...";
  if (segmented_objects_topic_ != nullptr)
  {
    response->segmentation.objects_vector = segmented_objects_topic_->objects_vector;
  }
}
void Outputs::RosServiceOutput::setServiceResponse(
    boost::shared_ptr<vino_people_msgs::ReidentificationSrv::Response> response)
{
  slog::info << "in Reidentification service::Response ...";
  if (person_reid_topic_ != nullptr)
  {
    response->reidentification.reidentified_vector = person_reid_topic_->reidentified_vector;
  }
}

void Outputs::RosServiceOutput::setServiceResponse(boost::shared_ptr<vino_people_msgs::PeopleSrv::Response> response)
{
  slog::info << "in People::Response ...";
  if (faces_topic_ != nullptr)
  {
    slog::info << "[FACES],";
    response->persons.faces = faces_topic_->objects_vector;
  }
  else if (detected_objects_topic_ != nullptr)
  {
    slog::info << "[FACES(objects)],";
    response->persons.faces = detected_objects_topic_->objects_vector;
  }
  if (age_gender_topic_ != nullptr)
  {
    slog::info << "[AGE_GENDER],";
    response->persons.agegenders = age_gender_topic_->objects;
  }
  if (emotions_topic_ != nullptr)
  {
    slog::info << "[EMOTION],";
    response->persons.emotions = emotions_topic_->emotions;
  }
  if (headpose_topic_ != nullptr)
  {
    slog::info << "[HEADPOSE],";
    response->persons.headposes = headpose_topic_->headposes;
  }
  slog::info << "DONE!" << slog::endl;
}

void Outputs::RosServiceOutput::clearData()
{
  faces_topic_ = nullptr;
  detected_objects_topic_ = nullptr;
  age_gender_topic_ = nullptr;
  emotions_topic_ = nullptr;
  headpose_topic_ = nullptr;
  // segmented_object_topic_ = nullptr;
  person_reid_topic_ = nullptr;
}
