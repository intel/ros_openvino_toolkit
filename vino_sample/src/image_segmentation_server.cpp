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

#include <unistd.h>
#include <algorithm>
#include <chrono>
#include <csignal>
#include <fstream>
#include <functional>
#include <iostream>
#include <iterator>
#include <map>
#include <memory>
#include <random>
#include <string>
#include <thread>
#include <vector>
#include <utility>

#include <vino_param_lib/param_manager.h>
#include "vino_core_lib/common.h"
#include "vino_core_lib/engines/engine.h"
#include "vino_core_lib/inferences/age_gender_detection.h"
#include "vino_core_lib/inferences/base_inference.h"
#include "vino_core_lib/inferences/emotions_detection.h"
#include "vino_core_lib/inferences/face_detection.h"
#include "vino_core_lib/inferences/head_pose_detection.h"
#include "vino_core_lib/inputs/realsense_camera_topic.h"
#include "vino_core_lib/outputs/image_window_output.h"
#include "vino_core_lib/outputs/ros_topic_output.h"
#include "vino_core_lib/outputs/rviz_output.h"
#include "vino_core_lib/pipeline.h"
#include "vino_core_lib/pipeline_manager.h"
#include "vino_core_lib/slog.h"
#include "openvino/openvino.hpp"
#include "opencv2/opencv.hpp"
#include "vino_sample/utility.hpp"

bool parseAndCheckCommandLine(int argc, char** argv)
{
  // -----Parsing and validation of input args---------------------------
  gflags::ParseCommandLineNonHelpFlags(&argc, &argv, true);
  if (FLAGS_h)
  {
    showUsageForParam();
    return false;
  }

  return true;
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "image_segmentation_servier");

  if (!parseAndCheckCommandLine(argc, argv))
    return 0;

  ros::param::param<std::string>("~param_file", FLAGS_config, "/param/image_segmentation_server.yaml");

  slog::info << "FLAGS_config=" << FLAGS_config << slog::endl;

  std::string service_name = "/openvino_toolkit/service";
  slog::info << "service name=" << service_name << slog::endl;
  // ----- Parsing and validation of input args-----------------------

  auto node =
      std::make_shared<vino_service::FrameProcessingServer<vino_people_msgs::ObjectsInMasksSrv>>(service_name, FLAGS_config);

  slog::info << "Waiting for seg service request..." << slog::endl;
  ros::spin();
  slog::info << "--------------End of Excution--------------" << FLAGS_config << slog::endl;
}
