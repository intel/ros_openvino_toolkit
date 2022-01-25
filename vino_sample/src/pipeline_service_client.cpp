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
* \brief A sample for this library. This sample performs face detection,
 * emotions detection, age gender detection and head pose estimation.
* \file sample/main.cpp
*/

#include <ros/package.h>
#include <ros/ros.h>
#include "opencv2/opencv.hpp"

#include <vino_pipeline_srv_msgs/PipelineSrv.h>
#include <vino_pipeline_srv_msgs/Pipelines.h>

vino_pipeline_srv_msgs::PipelineSrv getRequestMsg(std::string cmd, std::string value)
{
  vino_pipeline_srv_msgs::PipelineSrv srv;
  srv.request.pipeline_request.cmd = cmd;
  srv.request.pipeline_request.value = value;
  return srv;
}

bool stopPipelineByName(ros::ServiceClient& client, std::string name)
{
  auto srv = getRequestMsg("STOP_PIPELINE", name);  // object_pipeline1
  bool result = client.call(srv) ? true : false;
  return result;
}

bool pausePipelineByName(ros::ServiceClient& client, std::string name)
{
  auto srv = getRequestMsg("PAUSE_PIPELINE", name);  // object_pipeline1
  bool result = client.call(srv) ? true : false;
  return result;
}

bool runPipelineByName(ros::ServiceClient& client, std::string name)
{
  auto srv = getRequestMsg("RUN_PIPELINE", name);  // object_pipeline1
  bool result = client.call(srv) ? true : false;
  return result;
}

bool request(ros::ServiceClient& client, std::string cmd_name, std::string cmd_value)
{
  auto srv = getRequestMsg(cmd_name, cmd_value);  // object_pipeline1
  bool result = client.call(srv) ? true : false;
  return result;
}

bool getAllPipelines(ros::ServiceClient& client, std::vector<vino_pipeline_srv_msgs::Pipelines>& pipelines)
{
  auto srv = getRequestMsg("GET_PIPELINE", "");
  bool success = client.call(srv) ? true : false;
  if (success)
  {
    for (auto it = srv.response.pipelines.begin(); it != srv.response.pipelines.end(); ++it)
    {
      pipelines.push_back(*it);
      std::cout << it->name << " status:" << it->running_status << std::endl;
      for (auto connect = it->connections.begin(); connect != it->connections.end(); ++connect)
      {
        printf("%s --> %s\n", connect->input.c_str(), connect->output.c_str());
      }
    }
  }
  return success;
}

int main(int argc, char** argv)
{
  if (argc != 3)
  {
    ROS_INFO("Usage: rosrun vino_sample pipeline_service_client <cmd_name> <cmd_value>");
    return -1;
  }
  std::string cmd_name = argv[1];
  std::string cmd_value = argv[2];

  ros::init(argc, argv, "pipeline_service_client");

  ros::NodeHandle n;

  ros::ServiceClient client = n.serviceClient<vino_pipeline_srv_msgs::PipelineSrv>("/openvino_toolkit/pipeline/service");

  std::vector<vino_pipeline_srv_msgs::Pipelines> pipelines;

  auto success = getAllPipelines(client, pipelines) ? true : false;

  if (!success)
  {
    ROS_ERROR("Failed to request service.");
    return -1;
  }

  success = request(client, cmd_name, cmd_value);
  if (!success)
  {
    ROS_ERROR("Failed to request service.");
    return -1;
  }
  // stopPipelineByName(client,"object_pipeline1");
}
