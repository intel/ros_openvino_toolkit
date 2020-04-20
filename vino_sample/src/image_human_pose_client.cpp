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
* \brief A sample for this library. This sample performs human pose estimation.
* \file src/image_human_pose_client.cpp
*/

#include <ros/package.h>
#include <ros/ros.h>
#include "opencv2/opencv.hpp"

#include <vino_people_msgs/HumanPoseSrv.h>



int main(int argc, char ** argv)
{
  ros::init(argc, argv, "image_human_pose_client"); 

  ros::NodeHandle n;

  ros::ServiceClient client = n.serviceClient<vino_people_msgs::HumanPoseSrv>("/openvino_toolkit/service");


  vino_people_msgs::HumanPoseSrv srv;

  if (client.call(srv))
  {
    ROS_INFO("Request service success!"); 

    for (unsigned int i = 0; i < srv.response.humanposes.size(); i++) {
      std::ostringstream os;
      os << "Pose " << i << ", keypoints (" << srv.response.humanposes[i].keypoints.size() << "):";
      
      for (auto kp : srv.response.humanposes[i].keypoints)
      {
          os << std::endl<< "\tx: " << kp.x << "\ty: " << kp.y << "\tscore: " << kp.score;
      }
      ROS_INFO(os.str().c_str());
    }
   
   
  }
   else 
  {
    ROS_ERROR("Failed to request service \"openvino_toolkit/service\" "); return -1;
  }

    
   

}


 
