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
 * @brief a header file with declaration of Pipeline Manager class
 * @file pipeline_manager.cpp
 */

#include <memory>
#include <string>
#include <utility>
#include <vino_param_lib/param_manager.h>

#include "vino_core_lib/inferences/age_gender_detection.h"
#include "vino_core_lib/inferences/emotions_detection.h"
#include "vino_core_lib/inferences/face_detection.h"
#include "vino_core_lib/inferences/head_pose_detection.h"
#include "vino_core_lib/inferences/face_reidentification.h"
#include "vino_core_lib/inferences/person_attribs_detection.h"
#include "vino_core_lib/inferences/vehicle_attribs_detection.h"
#include "vino_core_lib/inferences/license_plate_detection.h"
#include "vino_core_lib/inferences/landmarks_detection.h"

#include "vino_core_lib/inputs/image_input.h"
#include "vino_core_lib/inputs/realsense_camera.h"
#include "vino_core_lib/inputs/realsense_camera_topic.h"
#include "vino_core_lib/inputs/standard_camera.h"
#include "vino_core_lib/inputs/ip_camera.h"
#include "vino_core_lib/inputs/video_input.h"

#include "vino_core_lib/models/age_gender_detection_model.h"
#include "vino_core_lib/models/emotion_detection_model.h"
#include "vino_core_lib/models/face_detection_model.h"
#include "vino_core_lib/models/head_pose_detection_model.h"
#include "vino_core_lib/models/object_detection_ssd_model.h"
// #include "vino_core_lib/models/object_detection_yolov2voc_model.h"
#include "vino_core_lib/models/face_reidentification_model.h"
#include "vino_core_lib/models/person_attribs_detection_model.h"
#include "vino_core_lib/models/vehicle_attribs_detection_model.h"
#include "vino_core_lib/models/license_plate_detection_model.h"
#include "vino_core_lib/models/landmarks_detection_model.h"

#include "vino_core_lib/outputs/image_window_output.h"
#include "vino_core_lib/outputs/ros_topic_output.h"
#include "vino_core_lib/outputs/rviz_output.h"
#include "vino_core_lib/outputs/ros_service_output.h"

#include "vino_core_lib/pipeline.h"
#include "vino_core_lib/pipeline_manager.h"
#include "vino_core_lib/pipeline_params.h"
#include "vino_core_lib/services/pipeline_processing_server.h"
#include "vino_core_lib/engines/engine_manager.h"

std::shared_ptr<Pipeline> PipelineManager::createPipeline(const Params::ParamManager::PipelineRawData& params)
{
  if (params.name == "")
  {
    throw std::logic_error("The name of pipeline won't be empty!");
  }

  std::shared_ptr<Pipeline> pipeline = std::make_shared<Pipeline>(params.name);
  pipeline->getParameters()->update(params);

  PipelineData data;
  data.pipeline = pipeline;
  data.params = params;
  data.state = PipelineState_ThreadNotCreated;

  auto inputs = parseInputDevice(data);
  if (inputs.size() != 1)
  {
    slog::err << "currently one pipeline only supports ONE input." << slog::endl;
    return nullptr;
  }
  for (auto it = inputs.begin(); it != inputs.end(); ++it)
  {
    pipeline->add(it->first, it->second);
    auto node = it->second->getHandler();
    if (node != nullptr)
    {
      data.spin_nodes.emplace_back(node);
    }
  }

  auto outputs = parseOutput(data);
  for (auto it = outputs.begin(); it != outputs.end(); ++it)
  {
    pipeline->add(it->first, it->second);
  }

  auto infers = parseInference(params);
  for (auto it = infers.begin(); it != infers.end(); ++it)
  {
    pipeline->add(it->first, it->second);
  }

  slog::info << "Updating connections ..." << slog::endl;
  for (auto it = params.connects.begin(); it != params.connects.end(); ++it)
  {
    pipeline->add(it->first, it->second);
  }

  pipelines_.insert({ params.name, data });

  pipeline->setCallback();
  slog::info << "One Pipeline Created!" << slog::endl;
  pipeline->printPipeline();
  return pipeline;
}

std::map<std::string, std::shared_ptr<Input::BaseInputDevice>>
PipelineManager::parseInputDevice(const PipelineData& pdata)
{
  std::map<std::string, std::shared_ptr<Input::BaseInputDevice>> inputs;

  for (auto& input_name : pdata.params.inputs)
  {
    if (input_name.empty())
    {
      continue;
    }

    slog::info << "Parsing InputDvice: " << input_name << slog::endl;

    std::shared_ptr<Input::BaseInputDevice> device = REG_INPUT_FACTORY::produce_shared(input_name);

    if (device != nullptr)
    {
      device->init(pdata.params.input_meta);
      inputs.insert({ input_name, device });
      slog::info << " ... Adding one Input device: " << input_name << slog::endl;
    }
    else
    {
      slog::err << "Invalid input device input_name: " << input_name << slog::endl;
    }
  }

  return inputs;
}

std::map<std::string, std::shared_ptr<Outputs::BaseOutput>> PipelineManager::parseOutput(const PipelineData& pdata)
{
  std::map<std::string, std::shared_ptr<Outputs::BaseOutput>> outputs;
  for (auto& output_name : pdata.params.outputs)
  {
    if (output_name.empty())
    {
      continue;
    }

    slog::info << "Parsing Output: " << output_name << slog::endl;

    std::shared_ptr<Outputs::BaseOutput> output = REG_OUTPUT_FACTORY::produce_shared(output_name);
    
    if (output != nullptr)
    {
      output->init(pdata.params.name);
      outputs.insert({ output_name, output });
      slog::info << " ... Adding one Output: " << output_name << slog::endl;
    }
    else
    {
      slog::err << "Invalid output output_name: " << output_name << slog::endl;
    }
  }

  return outputs;
}

std::map<std::string, std::shared_ptr<vino_core_lib::BaseInference>>
PipelineManager::parseInference(const Params::ParamManager::PipelineRawData& params)
{
  std::map<std::string, std::shared_ptr<vino_core_lib::BaseInference>> inferences;

  for (auto& infer_param : params.infers)
  {
    if (infer_param.name.empty() || infer_param.model.empty())
    {
      continue;
    }

    // Model
    std::shared_ptr<Models::BaseModel> model = nullptr;

    if(infer_param.model_type.empty() && (infer_param.name.empty()))
    {
      slog::err << "Model" << infer_param.name << " init failed!!! Please input model name or type.";
      // return inferences;
    }

    if(!(infer_param.name.empty()))
    {
      auto key_name = infer_param.name;

      if(!(infer_param.model_type.empty()))
      {
        key_name = key_name + "_" + infer_param.model_type;
      }

      model = REG_MODEL_FACTORY::produce_shared(key_name);
    }

    // void Models::BaseModel::init(const std::string& label_loc, const std::string& model_loc, int max_batch_size)

      slog::debug << "[InferenceRawData]" << slog::endl 
                  << "Name: " << infer_param.name << slog::endl 
                  << "Engine: " << infer_param.engine  << slog::endl 
                  << "Model: " << infer_param.model  << slog::endl 
                  << "Model_type: " << infer_param.model_type  << slog::endl 
                  << "Label: " << infer_param.label  << slog::endl 
                  << "Batch: " << infer_param.batch  << slog::endl 
                  << "Confidence_threshold: " << infer_param.confidence_threshold  << slog::endl 
                  << "Enable_roi_constraint: " << infer_param.enable_roi_constraint  << slog::endl;

    if(model != nullptr)
    {
      model->init(infer_param.label, infer_param.model, infer_param.batch);
    }
    else
    {
      slog::err << "Model produce failed!!!" << slog::endl;
    }

    // Engine
    auto engine = engine_manager_.createEngine(infer_param.engine, model);

    slog::info << "Parsing Inference: " << infer_param.name << slog::endl;
    auto infer = REG_INFERENCE_FACTORY::produce_shared(infer_param.name);

    if(infer != nullptr)
    {
      infer->loadNetwork(model); 
      infer->loadEngine(engine);
      inferences.insert({ infer_param.name, infer});
      slog::info << " ... Adding one Inference: " << infer_param.name << slog::endl;
    }
  }

  return inferences;
}

void PipelineManager::threadPipeline(const char* name)
{
  PipelineData& p = pipelines_[name];
  while (p.state != PipelineState_ThreadStopped && p.pipeline != nullptr && ros::ok())
  {
    if (p.state != PipelineState_ThreadPasued)
    {
      ros::spinOnce();
      p.pipeline->runOnce();
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }
}

void PipelineManager::runAll()
{
  for (auto it = pipelines_.begin(); it != pipelines_.end(); ++it)
  {
    if (it->second.state != PipelineState_ThreadRunning)
    {
      it->second.state = PipelineState_ThreadRunning;
    }
    if (it->second.thread == nullptr)
    {
      it->second.thread =
          std::make_shared<std::thread>(&PipelineManager::threadPipeline, this, it->second.params.name.c_str());
    }
  }
}

void PipelineManager::stopAll()
{
  for (auto it = pipelines_.begin(); it != pipelines_.end(); ++it)
  {
    if (it->second.state == PipelineState_ThreadRunning)
    {
      it->second.state = PipelineState_ThreadStopped;
    }
  }
}

void PipelineManager::runService()
{
  auto node =
      std::make_shared<vino_service::PipelineProcessingServer<vino_pipeline_srv_msgs::PipelineSrv>>("pipeline_service");
  ros::spin();  // hold the thread waiting for pipeline service
}

void PipelineManager::joinAll()
{
  auto service_thread = std::make_shared<std::thread>(&PipelineManager::runService, this);
  service_thread->join();  // pipeline service

  for (auto it = pipelines_.begin(); it != pipelines_.end(); ++it)
  {
    if (it->second.thread != nullptr && it->second.state == PipelineState_ThreadRunning)
    {
      it->second.thread->join();
    }
  }
}
