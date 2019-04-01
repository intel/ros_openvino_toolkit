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
#include "dynamic_vino_lib/common.h"
#include "dynamic_vino_lib/engines/engine.h"
#include "dynamic_vino_lib/factory.h"
#include "dynamic_vino_lib/inferences/age_gender_detection.h"
#include "dynamic_vino_lib/inferences/base_inference.h"
#include "dynamic_vino_lib/inferences/emotions_detection.h"
#include "dynamic_vino_lib/inferences/face_detection.h"
#include "dynamic_vino_lib/inferences/head_pose_detection.h"
#include "dynamic_vino_lib/inputs/realsense_camera_topic.h"
#include "dynamic_vino_lib/outputs/image_window_output.h"
#include "dynamic_vino_lib/outputs/ros_topic_output.h"
#include "dynamic_vino_lib/outputs/rviz_output.h"
#include "dynamic_vino_lib/pipeline.h"
#include "dynamic_vino_lib/pipeline_manager.h"
#include "dynamic_vino_lib/slog.h"
#include "inference_engine.hpp"
#include "opencv2/opencv.hpp"
#include "sample/utility.hpp"

void signalHandler(int signum)
{
  slog::warn << "!!!!!!!!!!!Interrupt signal (" << signum << ") received!!!!!!!!!!!!" << slog::endl;

  PipelineManager::getInstance().stopAll();
}

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

  ros::init(argc, argv, "sample_with_params"); 
  
  // register signal SIGINT and signal handler
  signal(SIGINT, signalHandler);

  std::string FLAGS_config;
  ros::param::param<std::string>("~param_file", FLAGS_config, "/param/pipeline_people.yaml");
  slog::info << "FLAGS_config=" << FLAGS_config << slog::endl;

  try
  {
    std::cout << "InferenceEngine: "
              << InferenceEngine::GetInferenceEngineVersion() << std::endl;

    // ----- Parsing and validation of input args-----------------------
    if (!parseAndCheckCommandLine(argc, argv))
    {
      return 0;
    }

    Params::ParamManager::getInstance().parse(FLAGS_config);
    Params::ParamManager::getInstance().print();

    auto pcommon = Params::ParamManager::getInstance().getCommon();
    auto pipelines = Params::ParamManager::getInstance().getPipelines();
    if (pipelines.size() < 1)
    {
      throw std::logic_error("Pipeline parameters should be set!");
    }

    for (auto & p : pipelines) {
      PipelineManager::getInstance().createPipeline(p);
    }
   
    PipelineManager::getInstance().runAll();
    PipelineManager::getInstance().joinAll();

    slog::info << "Execution successful" << slog::endl;
   
  }
  catch (const std::exception& error)
  {
    slog::err << error.what() << slog::endl;
    return 1;
  }
  catch (...)
  {
    slog::err << "Unknown/internal exception happened." << slog::endl;
    return 1;
  }
}
