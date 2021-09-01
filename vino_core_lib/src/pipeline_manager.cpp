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
  for (auto& name : pdata.params.inputs)
  {
    slog::info << "Parsing InputDvice: " << name << slog::endl;
    std::shared_ptr<Input::BaseInputDevice> device = nullptr;
    if (name == kInputType_RealSenseCamera)
    {
      device = std::make_shared<Input::RealSenseCamera>();
    }
    else if (name == kInputType_StandardCamera)
    {
      device = std::make_shared<Input::StandardCamera>();
    }
    else if (name == kInputType_IpCamera)
    {
      if (pdata.params.input_meta != "")
      {
        device = std::make_shared<Input::IpCamera>(pdata.params.input_meta);
      }
    }
    else if (name == kInputType_CameraTopic || name == kInputType_ImageTopic)
    {
      device = std::make_shared<Input::ImageTopic>();
    }
    else if (name == kInputType_Video)
    {
      if (pdata.params.input_meta != "")
      {
        device = std::make_shared<Input::Video>(pdata.params.input_meta);
      }
    }
    else if (name == kInputType_Image)
    {
      if (pdata.params.input_meta != "")
      {
        device = std::make_shared<Input::Image>(pdata.params.input_meta);
      }
    }
    else
    {
      slog::err << "Invalid input device name: " << name << slog::endl;
    }

    if (device != nullptr)
    {
      device->initialize();
      inputs.insert({ name, device });
      slog::info << " ... Adding one Input device: " << name << slog::endl;
    }
  }

  return inputs;
}

std::map<std::string, std::shared_ptr<Outputs::BaseOutput>> PipelineManager::parseOutput(const PipelineData& pdata)
{
  std::map<std::string, std::shared_ptr<Outputs::BaseOutput>> outputs;
  for (auto& name : pdata.params.outputs)
  {
    slog::info << "Parsing Output: " << name << slog::endl;
    std::shared_ptr<Outputs::BaseOutput> object = nullptr;
    if (name == kOutputTpye_RosTopic)
    {
      object = std::make_shared<Outputs::RosTopicOutput>(pdata.params.name);
    }
    else if (name == kOutputTpye_ImageWindow)
    {
      object = std::make_shared<Outputs::ImageWindowOutput>(pdata.params.name);
    }
    else if (name == kOutputTpye_RViz)
    {
      object = std::make_shared<Outputs::RvizOutput>(pdata.params.name);
    }
    else if (name == kOutputTpye_RosService)
    {
      object = std::make_shared<Outputs::RosServiceOutput>(pdata.params.name);
    }
    else
    {
      slog::err << "Invalid output name: " << name << slog::endl;
    }
    if (object != nullptr)
    {
      outputs.insert({ name, object });
      slog::info << " ... Adding one Output: " << name << slog::endl;
    }
  }

  return outputs;
}

std::map<std::string, std::shared_ptr<vino_core_lib::BaseInference>>
PipelineManager::parseInference(const Params::ParamManager::PipelineRawData& params)
{
  std::map<std::string, std::shared_ptr<vino_core_lib::BaseInference>> inferences;
  for (auto& infer : params.infers)
  {
    if (infer.name.empty() || infer.model.empty())
    {
      continue;
    }
    slog::info << "Parsing Inference: " << infer.name << slog::endl;
    std::shared_ptr<vino_core_lib::BaseInference> object = nullptr;

    if (infer.name == kInferTpye_FaceDetection)
    {
      object = createFaceDetection(infer);
    }
    else if (infer.name == kInferTpye_AgeGenderRecognition)
    {
      object = createAgeGenderRecognition(infer);
    }
    else if (infer.name == kInferTpye_EmotionRecognition)
    {
      object = createEmotionRecognition(infer);
    }
    else if (infer.name == kInferTpye_HeadPoseEstimation)
    {
      object = createHeadPoseEstimation(infer);
    }
    else if (infer.name == kInferTpye_ObjectDetection)
    {
      object = createObjectDetection(infer);
    }
    else if (infer.name == kInferTpye_ObjectSegmentation)
    {
      object = createObjectSegmentation(infer);
    }
    else if (infer.name == kInferTpye_PersonReidentification)
    {
      object = createPersonReidentification(infer);
    }
    else if (infer.name == kInferTpye_FaceReidentification)
    {
      object = createFaceReidentification(infer);
    }
    else if (infer.name == kInferTpye_PersonAttribsDetection)
    {
      object = createPersonAttribsDetection(infer);
    }
    else if (infer.name == kInferTpye_LandmarksDetection)
    {
      object = createLandmarksDetection(infer);
    }
    else if (infer.name == kInferTpye_VehicleAttribsDetection)
    {
      object = createVehicleAttribsDetection(infer);
    }
    else if (infer.name == kInferTpye_LicensePlateDetection)
    {
      object = createLicensePlateDetection(infer);
    }
    else
    {
      slog::err << "Invalid inference name: " << infer.name << slog::endl;
    }

    if (object != nullptr)
    {
      inferences.insert({ infer.name, object });
      slog::info << " ... Adding one Inference: " << infer.name << slog::endl;
    }
  }

  return inferences;
}

std::shared_ptr<vino_core_lib::BaseInference>
PipelineManager::createFaceDetection(const Params::ParamManager::InferenceRawData& infer)
{
  return createObjectDetection(infer);
}

std::shared_ptr<vino_core_lib::BaseInference>
PipelineManager::createAgeGenderRecognition(const Params::ParamManager::InferenceRawData& param)
{
  auto model = std::make_shared<Models::AgeGenderDetectionModel>(param.model, param.batch);
  model->modelInit();
  auto engine = engine_manager_.createEngine(param.engine, model);
  auto infer = std::make_shared<vino_core_lib::AgeGenderDetection>();
  infer->loadNetwork(model);
  infer->loadEngine(engine);

  return infer;
}

std::shared_ptr<vino_core_lib::BaseInference>
PipelineManager::createEmotionRecognition(const Params::ParamManager::InferenceRawData& param)
{
  auto model = std::make_shared<Models::EmotionDetectionModel>(param.model, param.batch);
  model->modelInit();
  auto engine = engine_manager_.createEngine(param.engine, model);
  auto infer = std::make_shared<vino_core_lib::EmotionsDetection>();
  infer->loadNetwork(model);
  infer->loadEngine(engine);

  return infer;
}

std::shared_ptr<vino_core_lib::BaseInference>
PipelineManager::createHeadPoseEstimation(const Params::ParamManager::InferenceRawData& param)
{
  auto model = std::make_shared<Models::HeadPoseDetectionModel>(param.model, param.batch);
  model->modelInit();
  auto engine = engine_manager_.createEngine(param.engine, model);
  auto infer = std::make_shared<vino_core_lib::HeadPoseDetection>();
  infer->loadNetwork(model);
  infer->loadEngine(engine);

  return infer;
}

std::shared_ptr<vino_core_lib::BaseInference>
PipelineManager::createObjectDetection(const Params::ParamManager::InferenceRawData& infer)
{
  std::shared_ptr<Models::ObjectDetectionModel> object_detection_model;
  std::shared_ptr<vino_core_lib::ObjectDetection> object_inference_ptr;
  slog::debug << "for test in createObjectDetection(), model_path =" << infer.model << slog::endl;
  if (infer.model_type == kInferTpye_ObjectDetectionTypeSSD)
  {
    object_detection_model = std::make_shared<Models::ObjectDetectionSSDModel>(infer.model, infer.batch);
  }
  // if (infer.model_type == kInferTpye_ObjectDetectionTypeYolov2voc)
  // {
  //   object_detection_model = std::make_shared<Models::ObjectDetectionYolov2Model>(infer.model, infer.batch);
  // }

  slog::debug << "for test in createObjectDetection(), Created SSDModel" << slog::endl;
  object_inference_ptr = std::make_shared<vino_core_lib::ObjectDetection>(
      infer.enable_roi_constraint, infer.confidence_threshold);  // To-do theshold configuration
  slog::debug << "for test in createObjectDetection(), before modelInit()" << slog::endl;
  object_detection_model->modelInit();
  auto object_detection_engine = engine_manager_.createEngine(infer.engine, object_detection_model);
  object_inference_ptr->loadNetwork(object_detection_model);
  object_inference_ptr->loadEngine(object_detection_engine);

  return object_inference_ptr;
}

std::shared_ptr<vino_core_lib::BaseInference>
PipelineManager::createObjectSegmentation(const Params::ParamManager::InferenceRawData& infer)
{
  auto model = std::make_shared<Models::ObjectSegmentationModel>(infer.model, infer.batch);
  model->modelInit();
  slog::info << "Segmentation model initialized." << slog::endl;
  auto engine = engine_manager_.createEngine(infer.engine, model);
  slog::info << "Segmentation Engine initialized." << slog::endl;
  auto segmentation_inference_ptr = std::make_shared<vino_core_lib::ObjectSegmentation>(infer.confidence_threshold);
  slog::info << "Segmentation Inference instanced." << slog::endl;
  segmentation_inference_ptr->loadNetwork(model);
  segmentation_inference_ptr->loadEngine(engine);

  return segmentation_inference_ptr;
}

std::shared_ptr<vino_core_lib::BaseInference>
PipelineManager::createPersonReidentification(const Params::ParamManager::InferenceRawData& infer)
{
  std::shared_ptr<Models::PersonReidentificationModel> person_reidentification_model;
  std::shared_ptr<vino_core_lib::PersonReidentification> reidentification_inference_ptr;
  slog::debug << "for test in createPersonReidentification()" << slog::endl;
  person_reidentification_model = std::make_shared<Models::PersonReidentificationModel>(infer.model, infer.batch);
  person_reidentification_model->modelInit();
  slog::info << "Reidentification model initialized" << slog::endl;
  auto person_reidentification_engine = engine_manager_.createEngine(infer.engine, person_reidentification_model);
  reidentification_inference_ptr =
      std::make_shared<vino_core_lib::PersonReidentification>(infer.confidence_threshold);
  reidentification_inference_ptr->loadNetwork(person_reidentification_model);
  reidentification_inference_ptr->loadEngine(person_reidentification_engine);

  return reidentification_inference_ptr;
}

std::shared_ptr<vino_core_lib::BaseInference>
PipelineManager::createVehicleAttribsDetection(const Params::ParamManager::InferenceRawData& infer)
{
  auto model = std::make_shared<Models::VehicleAttribsDetectionModel>(infer.model, infer.batch);
  model->modelInit();
  auto engine = engine_manager_.createEngine(infer.engine, model);
  auto vehicle_attribs_ptr = std::make_shared<vino_core_lib::VehicleAttribsDetection>();
  vehicle_attribs_ptr->loadNetwork(model);
  vehicle_attribs_ptr->loadEngine(engine);

  return vehicle_attribs_ptr;
}

std::shared_ptr<vino_core_lib::BaseInference>
PipelineManager::createLicensePlateDetection(const Params::ParamManager::InferenceRawData& infer)
{
  auto model = std::make_shared<Models::LicensePlateDetectionModel>(infer.model, infer.batch);
  model->modelInit();
  auto engine = engine_manager_.createEngine(infer.engine, model);
  auto license_plate_ptr = std::make_shared<vino_core_lib::LicensePlateDetection>();
  license_plate_ptr->loadNetwork(model);
  license_plate_ptr->loadEngine(engine);

  return license_plate_ptr;
}

std::shared_ptr<vino_core_lib::BaseInference>
PipelineManager::createPersonAttribsDetection(const Params::ParamManager::InferenceRawData& infer)
{
  auto model = std::make_shared<Models::PersonAttribsDetectionModel>(infer.model, infer.batch);
  slog::debug << "for test in createPersonAttributesDetection()" << slog::endl;
  model->modelInit();
  auto engine = engine_manager_.createEngine(infer.engine, model);
  auto attribs_inference_ptr = std::make_shared<vino_core_lib::PersonAttribsDetection>(infer.confidence_threshold);
  attribs_inference_ptr->loadNetwork(model);
  attribs_inference_ptr->loadEngine(engine);

  return attribs_inference_ptr;
}

std::shared_ptr<vino_core_lib::BaseInference>
PipelineManager::createFaceReidentification(const Params::ParamManager::InferenceRawData& infer)
{
  auto model = std::make_shared<Models::FaceReidentificationModel>(infer.model, infer.batch);
  slog::debug << "for test in createFaceReidentification()" << slog::endl;
  model->modelInit();
  auto engine = engine_manager_.createEngine(infer.engine, model);
  auto attribs_inference_ptr = std::make_shared<vino_core_lib::FaceReidentification>(infer.confidence_threshold);
  attribs_inference_ptr->loadNetwork(model);
  attribs_inference_ptr->loadEngine(engine);

  return attribs_inference_ptr;
}

std::shared_ptr<vino_core_lib::BaseInference>
PipelineManager::createLandmarksDetection(
  const Params::ParamManager::InferenceRawData & infer)
{
  auto model = std::make_shared<Models::LandmarksDetectionModel>(infer.model, infer.batch);
  model->modelInit();
  auto engine = engine_manager_.createEngine(infer.engine, model);
  auto landmarks_inference_ptr =  std::make_shared<vino_core_lib::LandmarksDetection>();
  landmarks_inference_ptr->loadNetwork(model);
  landmarks_inference_ptr->loadEngine(engine);

  return landmarks_inference_ptr;
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
