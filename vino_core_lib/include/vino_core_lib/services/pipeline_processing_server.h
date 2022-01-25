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
#ifndef VINO_CORE_LIB__SERVICES__PIPELINE_PROCESSING_SERVER_H
#define VINO_CORE_LIB__SERVICES__PIPELINE_PROCESSING_SERVER_H

#include <object_msgs/Object.h>
#include <object_msgs/ObjectInBox.h>
#include <object_msgs/ObjectsInBoxes.h>
#include <vino_people_msgs/Emotion.h>
#include <vino_people_msgs/EmotionsStamped.h>
#include <vino_people_msgs/AgeGender.h>
#include <vino_people_msgs/AgeGenderStamped.h>
#include <vino_people_msgs/HeadPose.h>
#include <vino_people_msgs/HeadPoseStamped.h>

#include <vino_people_msgs/AgeGenderSrv.h>
#include <vino_people_msgs/EmotionSrv.h>
#include <vino_people_msgs/HeadPoseSrv.h>
#include <vino_people_msgs/PeopleSrv.h>
#include <object_msgs/DetectObjectRequest.h>
#include <vino_people_msgs/ObjectsInMasksSrv.h>
#include <vino_people_msgs/ReidentificationSrv.h>
#include <vino_pipeline_srv_msgs/PipelineSrv.h>
#include "vino_core_lib/pipeline_manager.h"

#include <ros/ros.h>
#include <memory>
#include <iostream>
#include <string>

namespace vino_service
{
template <typename T>
class PipelineProcessingServer
{
public:
  explicit PipelineProcessingServer(const std::string& service_name);

private:
  void initPipelineService();

  std::shared_ptr<ros::NodeHandle> nh_;

  bool cbService(ros::ServiceEvent<typename T::Request, typename T::Response>& event);

  void setResponse(ros::ServiceEvent<typename T::Request, typename T::Response>& event);

  void setPipelineByRequest(std::string pipeline_name, PipelineManager::PipelineState state);

  std::shared_ptr<ros::ServiceServer> service_;

  std::map<std::string, PipelineManager::PipelineData>* pipelines_;

  std::string service_name_;
};
}  // namespace vino_service
#endif  // VINO_CORE_LIB__SERVICES__FRAME_PROCESSING_SERVER_H
