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

#include "dynamic_vino_lib/services/pipeline_processing_server.h"
#include <chrono>
#include <map>
#include <memory>
#include <object_msgs/DetectObjectRequest.h>
#include <people_msgs/ObjectsInMasksSrv.h>
#include <people_msgs/PeopleSrv.h>
#include <people_msgs/ReidentificationSrv.h>
#include <pipeline_srv_msgs/Pipeline.h>
#include <pipeline_srv_msgs/PipelineRequest.h>
#include <pipeline_srv_msgs/Pipelines.h>
#include <ros/ros.h>
#include <string>
#include <thread>
#include <vino_param_lib/param_manager.h>

#include "dynamic_vino_lib/inputs/base_input.h"
#include "dynamic_vino_lib/inputs/image_input.h"
#include "dynamic_vino_lib/pipeline.h"
#include "dynamic_vino_lib/pipeline_manager.h"
#include "dynamic_vino_lib/slog.h"

namespace vino_service
{
template <typename T>
PipelineProcessingServer<T>::PipelineProcessingServer(const std::string& service_name) : service_name_(service_name)
{
  nh_ = std::make_shared<ros::NodeHandle>();
  pipelines_ = PipelineManager::getInstance().getPipelinesPtr();
  initPipelineService();
}

template <typename T>
void PipelineProcessingServer<T>::initPipelineService()
{
  ros::ServiceServer srv = nh_->advertiseService<
      ros::ServiceEvent<pipeline_srv_msgs::PipelineSrv::Request, pipeline_srv_msgs::PipelineSrv::Response>>(
      "/openvino_toolkit/pipeline/service",
      std::bind(&PipelineProcessingServer::cbService, this, std::placeholders::_1));
  service_ = std::make_shared<ros::ServiceServer>(srv);
}

template <typename T>
void PipelineProcessingServer<T>::setResponse(ros::ServiceEvent<typename T::Request, typename T::Response>& event)
{
  for (auto it = pipelines_->begin(); it != pipelines_->end(); ++it)
  {
    pipeline_srv_msgs::Pipelines pipeline_msg;
    pipeline_msg.name = it->first;
    pipeline_msg.running_status = std::to_string(it->second.state);

    auto connection_map = it->second.pipeline->getPipelineDetail();
    for (auto& current_pipe : connection_map)
    {
      pipeline_srv_msgs::Pipeline connection;
      connection.input = current_pipe.first.c_str();
      connection.output = current_pipe.second.c_str();
      pipeline_msg.connections.push_back(connection);
    }
    event.getResponse().pipelines.push_back(pipeline_msg);
  }
}

template <typename T>
void PipelineProcessingServer<T>::setPipelineByRequest(std::string pipeline_name, PipelineManager::PipelineState state)
{
  for (auto it = pipelines_->begin(); it != pipelines_->end(); ++it)
  {
    if (pipeline_name == it->first)
    {
      it->second.state = state;
    }
  }
}

template <typename T>
bool PipelineProcessingServer<T>::cbService(ros::ServiceEvent<typename T::Request, typename T::Response>& event)
{
  std::string req_cmd = event.getRequest().pipeline_request.cmd;
  std::string req_val = event.getRequest().pipeline_request.value;
  slog::info << "[PipelineProcessingServer] Pipeline Service get request cmd: " << req_cmd << " val:" << req_val
             << slog::endl;

  PipelineManager::PipelineState state;
  if (req_cmd != "GET_PIPELINE")  // not only get pipeline but also set pipeline by request
  {
    if (req_cmd == "STOP_PIPELINE")
      state = PipelineManager::PipelineState_ThreadStopped;
    else if (req_cmd == "RUN_PIPELINE")
      state = PipelineManager::PipelineState_ThreadRunning;
    else if (req_cmd == "PAUSE_PIPELINE")
      state = PipelineManager::PipelineState_ThreadPasued;

    setPipelineByRequest(req_val, state);
  }
  else  // fill in pipeline status into service response
  {
    setResponse(event);
  }

  return true;
}
template class PipelineProcessingServer<pipeline_srv_msgs::PipelineSrv>;
}  // namespace vino_service
