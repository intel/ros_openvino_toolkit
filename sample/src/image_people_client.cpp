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

#include <people_msgs/PeopleSrv.h>



int main(int argc, char ** argv)
{
  ros::init(argc, argv, "image_people_client"); 
  ros::NodeHandle n;

  if (argc != 2) {
    ROS_INFO( "Usage: rosrun dynamic_vino_sample image_people_client"
      "<image_path>");
    return -1;
  }


  ros::ServiceClient client = n.serviceClient<people_msgs::PeopleSrv>("/openvino_toolkit/service");

  people_msgs::PeopleSrv srv;

  if (client.call(srv))
  {
    ROS_INFO("Request service success!"); 

    if (srv.response.persons.emotions.size() == 0 && srv.response.persons.agegenders.size() == 0 &&
        srv.response.persons.headposes.size() == 0)
    {
      ROS_INFO( "Get response, but no any person found.");
      return 0;
    }
      ROS_INFO( "Found persons...");

    for (unsigned int i = 0; i < srv.response.persons.faces.size(); i++) {
      ROS_INFO( "%d: object: %s", i,
        srv.response.persons.faces[i].object.object_name.c_str());
      ROS_INFO( "prob: %f",
        srv.response.persons.faces[i].object.probability);
      ROS_INFO( "location: (%d, %d, %d, %d)",
        srv.response.persons.faces[i].roi.x_offset, srv.response.persons.faces[i].roi.y_offset,
        srv.response.persons.faces[i].roi.width, srv.response.persons.faces[i].roi.height);
      ROS_INFO( "Emotions: %s",
        srv.response.persons.emotions[i].emotion.c_str());
      ROS_INFO( "Age: %f, Gender: %s",
        srv.response.persons.agegenders[i].age, srv.response.persons.agegenders[i].gender.c_str());
      ROS_INFO( "Yaw, Pitch and Roll for head pose is: (%f, %f, %f),",
        srv.response.persons.headposes[i].yaw, srv.response.persons.headposes[i].pitch,
        srv.response.persons.headposes[i].roll);
      }
  }
  else 
  {
     ROS_ERROR("Failed to request service \"/openvino_toolkit/service\" "); 
     return -1; 
  }
}


 
