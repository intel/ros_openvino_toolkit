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

#include <object_msgs/DetectObject.h>




int main(int argc, char ** argv)
{
  ros::init(argc, argv, "image_object_client"); 

  ros::NodeHandle n;

  if (argc != 2) {
    ROS_INFO("Usage: rosrun dynamic_vino_sample image_object_client <image_path>");
    //You can find a sample image in /opt/openvino_toolkit/ros_openvino_toolkit/data/images/car.png
    return -1;
  }

  std::string image_path = argv[1];
  ros::ServiceClient client = n.serviceClient<object_msgs::DetectObject>("/openvino_toolkit/service");


  object_msgs::DetectObject srv;

  if (client.call(srv))
  {
    ROS_INFO("Request service success!"); 

    cv::Mat image = cv::imread(image_path);
    int width = image.cols;
    int height = image.rows;

    for (unsigned int i = 0; i < srv.response.objects[0].objects_vector.size(); i++) {
      std::stringstream ss;
      ss << srv.response.objects[0].objects_vector[i].object.object_name << ": " <<
        srv.response.objects[0].objects_vector[i].object.probability * 100 << "%";
      ROS_INFO("%d: object: %s", i,
        srv.response.objects[0].objects_vector[i].object.object_name.c_str());
      ROS_INFO( "prob: %f",
        srv.response.objects[0].objects_vector[i].object.probability);
      ROS_INFO(
        "location: (%d, %d, %d, %d)",
        srv.response.objects[0].objects_vector[i].roi.x_offset, srv.response.objects[0].objects_vector[i].roi.y_offset,
        srv.response.objects[0].objects_vector[i].roi.width, srv.response.objects[0].objects_vector[i].roi.height);

      int xmin = srv.response.objects[0].objects_vector[i].roi.x_offset;
      int ymin = srv.response.objects[0].objects_vector[i].roi.y_offset;
      int w = srv.response.objects[0].objects_vector[i].roi.width;
      int h = srv.response.objects[0].objects_vector[i].roi.height;

      int xmax = ((xmin + w) < width) ? (xmin + w) : width;
      int ymax = ((ymin + h) < height) ? (ymin + h) : height;

      cv::Point left_top = cv::Point(xmin, ymin);
      cv::Point right_bottom = cv::Point(xmax, ymax);
      cv::rectangle(image, left_top, right_bottom, cv::Scalar(0, 255, 0), 1, 8, 0);
      cv::rectangle(image, cvPoint(xmin, ymin), cvPoint(xmax, ymin + 20), cv::Scalar(0, 255, 0),
        -1);
      cv::putText(image, ss.str(), cvPoint(xmin + 5, ymin + 20), cv::FONT_HERSHEY_PLAIN, 1,
        cv::Scalar(0, 0, 255), 1);
    }
    cv::imshow("image_detection", image);
    cv::waitKey(0);
    

  } else 
  {
    ROS_ERROR("Failed to request service \"openvino_toolkit/service\" "); return -1;
  }

      
   

}


 
