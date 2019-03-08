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
#include "dynamic_vino_lib/slog.h"
#include "inference_engine.hpp"
#include "librealsense2/rs.hpp"
#include "opencv2/opencv.hpp"
#include "sample/utility.hpp"

bool parseAndCheckCommandLine(int argc, char** argv)
{
  // ---------------------------Parsing and validation of input
  // args-----------------------------
  gflags::ParseCommandLineNonHelpFlags(&argc, &argv, true);
  if (FLAGS_h)
  {
    showUsageForParam();
    return false;
  }

  return true;
}

int main(int argc, char* argv[]) 
{
  ros::init(argc, argv, "sample_pipeline_object");
  ros::NodeHandle n;
  std::string content;
  std::string prefix_path;

  prefix_path = ros::package::getPath("vino_launch");
  FLAGS_config = prefix_path + "/param/pipeline_object.yaml";

  slog::info << "prefix_path=" << prefix_path << slog::endl;

  try 
  {
    std::cout << "InferenceEngine: " 
              << InferenceEngine::GetInferenceEngineVersion() << std::endl;
    // ------------------------------ Parsing and validation of input args
    // -------------------------
    if (!parseAndCheckCommandLine(argc, argv)) 
    {
      return 0;
    }

    Params::ParamManager::getInstance().parse(FLAGS_config);
    Params::ParamManager::getInstance().print();

    auto pcommon = Params::ParamManager::getInstance().getCommon();
    auto pipelines = Params::ParamManager::getInstance().getPipelines();
    if (pipelines.size() < 1) {
      throw std::logic_error("Pipeline parameters should be set!");
    }

    FLAGS_i = pipelines[0].inputs[0];
    FLAGS_d = pipelines[0].infers[0].engine;
    FLAGS_m = pipelines[0].infers[0].model;
    FLAGS_c = pcommon.custom_cldnn_library;

    // ----------- 1. Load Plugin for inference engine
    //std::unique_ptr<InferenceEngine::InferencePlugin> plugin = Factory::makePluginByName(
    //  FLAGS_d, FLAGS_l, FLAGS_c, FLAGS_pc);
    std::map<std::string, InferenceEngine::InferencePlugin> plugins_for_devices;
    std::vector<std::pair<std::string, std::string>> cmd_options =
    {
      {FLAGS_d, FLAGS_m},
      {FLAGS_d_ag, FLAGS_m_ag},
      {FLAGS_d_hp, FLAGS_m_hp},
      {FLAGS_d_em, FLAGS_m_em}
    };
    slog::info << "device_FACE:" << FLAGS_d << slog::endl;
    slog::info << "model_FACE:" << FLAGS_m << slog::endl;
    slog::info << "device_AG:" << FLAGS_d_ag << slog::endl;
    slog::info << "model_AG:" << FLAGS_m_ag << slog::endl;
    slog::info << "model_HeadPose:" << FLAGS_m_hp << slog::endl;
    slog::info << "device_HeadPose:" << FLAGS_d_hp << slog::endl;

    for (auto&& option : cmd_options)
    {
      auto device_name = option.first;
      auto network_name = option.second;
      if (device_name.empty() || network_name.empty())
      {
        continue;
      }
      if (plugins_for_devices.find(device_name) != plugins_for_devices.end())
      {
        continue;
      }
      plugins_for_devices[device_name] =
          *Factory::makePluginByName(device_name, FLAGS_l, FLAGS_c, FLAGS_pc);
    }
    
    // --------------------------- 2. Generate Input Device and Output
    // Device-----------------------
    slog::info << "Reading input" << slog::endl;
    auto input_ptr = Factory::makeInputDeviceByName(FLAGS_i, FLAGS_i_path);
 
    if (!input_ptr->initialize()) {
      throw std::logic_error("Cannot open input file or camera: " + FLAGS_i);
    }
    std::string window_name = "Results";
    auto output_ptr = std::make_shared<Outputs::ImageWindowOutput>(window_name);
    // --------------------------- 3. Generate Inference
    // Instance-----------------------------------
    // generate face detection inference
    auto model =
        std::make_shared<Models::ObjectDetectionModel>(FLAGS_m, 1, 1, 1);
    model->modelInit();
    auto engine = std::make_shared<Engines::Engine>(plugins_for_devices[FLAGS_d], model);
    auto object_detection_ptr =
        std::make_shared<dynamic_vino_lib::ObjectDetection>(FLAGS_t);
    object_detection_ptr->loadNetwork(model);
    object_detection_ptr->loadEngine(engine);
    slog::info << "Pass Inference Prep." << slog::endl;
    // ------- 4. Build Pipeline -------------
    Pipeline pipe;
    pipe.add("video_input", input_ptr);
    pipe.add("video_input", "object_detection", object_detection_ptr);
    pipe.add("object_detection", "video_output", output_ptr);
    auto ros_topic_output_ptr = std::make_shared<Outputs::RosTopicOutput>();
    auto rviz_output_ptr = std::make_shared<Outputs::RvizOutput>();
    pipe.add("object_detection", "ros_output", ros_topic_output_ptr);
    pipe.add("object_detection", "rviz_output", rviz_output_ptr);
    pipe.setCallback();
    pipe.printPipeline();
    
    slog::info << "Pass Pipeline Init." << slog::endl;

    // ------- 5. Run Pipeline -----------
    while (cv::waitKey(1) < 0 && ros::ok())
    {
      ros::spinOnce();
      pipe.runOnce(FLAGS_i);
    }

    slog::info << "Execution successful" << slog::endl;
    return 0;
  } catch (const std::exception& error) {
    slog::err << error.what() << slog::endl;
    return 1;
  } catch (...) {
    slog::err << "Unknown/internal exception happened." << slog::endl;
    return 1;
  }
}
