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

#include <people_msgs/ObjectsInMasksSrv.h>



int main(int argc, char ** argv)
{
  ros::init(argc, argv, "image_segmentation_client"); 

  ros::NodeHandle n;

  ros::ServiceClient client = n.serviceClient<people_msgs::ObjectsInMasksSrv>("/openvino_toolkit/service");


  people_msgs::ObjectsInMasksSrv srv;

  if (client.call(srv))
  {
    ROS_INFO("Request service success!"); 
    
    for (unsigned int i = 0; i < srv.response.segmentation.objects_vector.size(); i++) {
      std::stringstream ss;
      ss << srv.response.segmentation.objects_vector[i].object_name << ": " <<
        srv.response.segmentation.objects_vector[i].probability * 100 << "%";
      ROS_INFO("%d: object: %s", i,
        srv.response.segmentation.objects_vector[i].object_name.c_str());
      ROS_INFO( "prob: %f",
        srv.response.segmentation.objects_vector[i].probability);
      ROS_INFO(
        "location: (%d, %d, %d, %d)",
        srv.response.segmentation.objects_vector[i].roi.x_offset, srv.response.segmentation.objects_vector[i].roi.y_offset,
        srv.response.segmentation.objects_vector[i].roi.width,srv.response.segmentation.objects_vector[i].roi.height);
    }
   
  }
   else 
  {
    ROS_ERROR("Failed to request service \"openvino_toolkit/service\" "); return -1;
  }

    
   

}


 