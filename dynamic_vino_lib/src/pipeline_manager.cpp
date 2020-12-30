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
#include "dynamic_vino_lib/inferences/age_gender_detection.h"
#include "dynamic_vino_lib/inferences/emotions_detection.h"
#include "dynamic_vino_lib/inferences/face_detection.h"
#include "dynamic_vino_lib/inferences/head_pose_detection.h"
#include "dynamic_vino_lib/inferences/face_reidentification.h"
#include "dynamic_vino_lib/inferences/person_attribs_detection.h"
#include "dynamic_vino_lib/inferences/vehicle_attribs_detection.h"
#include "dynamic_vino_lib/inferences/license_plate_detection.h"
#include "dynamic_vino_lib/inferences/landmarks_detection.h"
#include "dynamic_vino_lib/inputs/image_input.h"
#include "dynamic_vino_lib/inputs/realsense_camera.h"
#include "dynamic_vino_lib/inputs/realsense_camera_topic.h"
#include "dynamic_vino_lib/inputs/standard_camera.h"
#include "dynamic_vino_lib/inputs/video_input.h"
#include "dynamic_vino_lib/models/age_gender_detection_model.h"
#include "dynamic_vino_lib/models/emotion_detection_model.h"
#include "dynamic_vino_lib/models/face_detection_model.h"
#include "dynamic_vino_lib/models/head_pose_detection_model.h"
#include "dynamic_vino_lib/models/object_detection_ssd_model.h"
#include "dynamic_vino_lib/models/object_detection_yolov2voc_model.h"
#include "dynamic_vino_lib/models/face_reidentification_model.h"
#include "dynamic_vino_lib/models/person_attribs_detection_model.h"
#include "dynamic_vino_lib/models/vehicle_attribs_detection_model.h"
#include "dynamic_vino_lib/models/license_plate_detection_model.h"
#include "dynamic_vino_lib/models/landmarks_detection_model.h"
#include "dynamic_vino_lib/outputs/image_window_output.h"
#include "dynamic_vino_lib/outputs/ros_topic_output.h"
#include "dynamic_vino_lib/outputs/rviz_output.h"
#include "dynamic_vino_lib/outputs/ros_service_output.h"
#include "dynamic_vino_lib/pipeline.h"
#include "dynamic_vino_lib/pipeline_manager.h"
#include "dynamic_vino_lib/pipeline_params.h"
#include "dynamic_vino_lib/services/pipeline_processing_server.h"
#include "dynamic_vino_lib/engines/engine_manager.h"

std::shared_ptr<Pipeline> PipelineManager::createPipeline(
    const Params::ParamManager::PipelineRawData& params) {
  if (params.name == "") {
    throw std::logic_error("The name of pipeline won't be empty!");
  }
  PipelineData data;

  std::shared_ptr<Pipeline> pipeline = std::make_shared<Pipeline>(params.name);
  pipeline->getParameters()->update(params);

  auto inputs = parseInputDevice(params);
  if (inputs.size() != 1) {
    slog::err << "currently one pipeline only supports ONE input."
              << slog::endl;
    return nullptr;
  }
  for (auto it = inputs.begin(); it != inputs.end(); ++it) {
    pipeline->add(it->first, it->second);
    auto node = it->second->getHandler();
    if(node != nullptr) {
      data.spin_nodes.emplace_back(node); 
    }
  }

  auto outputs = parseOutput(params);
  for (auto it = outputs.begin(); it != outputs.end(); ++it) {
    pipeline->add(it->first, it->second);
  }

  auto infers = parseInference(params);
  for (auto it = infers.begin(); it != infers.end(); ++it) {
    pipeline->add(it->first, it->second);
  }

  slog::info << "Updating connections ..." << slog::endl;
  for (auto it = params.connects.begin(); it != params.connects.end(); ++it) {
    pipeline->add(it->first, it->second);
  }

  data.pipeline = pipeline;
  data.params = params;
  data.state = PipelineState_ThreadNotCreated;
  pipelines_.insert({params.name, data});

  pipeline->setCallback();
  slog::info << "One Pipeline Created!" << slog::endl;
  pipeline->printPipeline();
  return pipeline;
}

std::map<std::string, std::shared_ptr<Input::BaseInputDevice>>
PipelineManager::parseInputDevice(
    const Params::ParamManager::PipelineRawData& params) {
  std::map<std::string, std::shared_ptr<Input::BaseInputDevice>> inputs;
  for (auto& name : params.inputs) {
    slog::info << "Parsing InputDvice: " << name << slog::endl;
    std::shared_ptr<Input::BaseInputDevice> device = nullptr;
    if (name == kInputType_RealSenseCamera) {
      device = std::make_shared<Input::RealSenseCamera>();
    } else if (name == kInputType_StandardCamera) {
      device = std::make_shared<Input::StandardCamera>();
    } else if (name == kInputType_CameraTopic) {
      device = std::make_shared<Input::RealSenseCameraTopic>();
      std::cout <<"register yaml"<<std::endl;
    } else if (name == kInputType_Video) {
      if (params.input_meta != "") {
        device = std::make_shared<Input::Video>(params.input_meta);
      }
    } else if (name == kInputType_Image) {
      if (params.input_meta != "") {
        device = std::make_shared<Input::Image>(params.input_meta);
      }
    }

    if (device != nullptr) {
      device->initialize();
      inputs.insert({name, device});
      slog::info << " ... Adding one Input device: " << name << slog::endl;
    }
  }

  return inputs;
}

std::map<std::string, std::shared_ptr<Outputs::BaseOutput>>
PipelineManager::parseOutput(
    const Params::ParamManager::PipelineRawData& params) {
  std::map<std::string, std::shared_ptr<Outputs::BaseOutput>> outputs;
  for (auto& name : params.outputs) {
    slog::info << "Parsing Output: " << name << slog::endl;
    std::shared_ptr<Outputs::BaseOutput> object = nullptr;
    if (name == kOutputTpye_RosTopic) {
      object = std::make_shared<Outputs::RosTopicOutput>(params.name);
    } else if (name == kOutputTpye_ImageWindow) {
      object = std::make_shared<Outputs::ImageWindowOutput>(params.name);
    } else if (name == kOutputTpye_RViz) {
      object = std::make_shared<Outputs::RvizOutput>(params.name);
    } else if (name == kOutputTpye_RosService) {
      object = std::make_shared<Outputs::RosServiceOutput>(params.name);
    } else {
      slog::err << "Invalid output name: " << name << slog::endl;
    }
    if (object != nullptr) {
      outputs.insert({name, object});
      slog::info << " ... Adding one Output: " << name << slog::endl;
    }
  }

  return outputs;
}

std::map<std::string, std::shared_ptr<dynamic_vino_lib::BaseInference>>
PipelineManager::parseInference(
    const Params::ParamManager::PipelineRawData& params) {
  /**< update plugins for devices >**/
  auto pcommon = Params::ParamManager::getInstance().getCommon();
  std::string FLAGS_l = pcommon.custom_cpu_library;
  std::string FLAGS_c = pcommon.custom_cldnn_library;
  //std::string FLAGS_c = "/opt/intel/computer_vision_sdk/deployment_tools/inference_engine/lib/ubuntu_16.04/intel64/cldnn_global_custom_kernels/cldnn_global_custom_kernels.xml";

  bool FLAGS_pc = pcommon.enable_performance_count;

  std::map<std::string, std::shared_ptr<dynamic_vino_lib::BaseInference>>
      inferences;
  for (auto& infer : params.infers) {
    if (infer.name.empty() || infer.model.empty()) {
      continue;
    }
    slog::info << "Parsing Inference: " << infer.name << slog::endl;
    std::shared_ptr<dynamic_vino_lib::BaseInference> object = nullptr;

    if (infer.name == kInferTpye_FaceDetection) {
      object = createFaceDetection(infer);
    } 
    else if (infer.name == kInferTpye_AgeGenderRecognition) {
      object = createAgeGenderRecognition(infer);
    } 
    else if (infer.name == kInferTpye_EmotionRecognition) {
      object = createEmotionRecognition(infer);
    } 
    else if (infer.name == kInferTpye_HeadPoseEstimation) {
      object = createHeadPoseEstimation(infer);
    } 
    else if (infer.name == kInferTpye_ObjectDetection) {
      object = createObjectDetection(infer);
    }
    else if (infer.name == kInferTpye_ObjectSegmentation) {
      object = createObjectSegmentation(infer);
    }
    else if (infer.name == kInferTpye_PersonReidentification) {
      object = createPersonReidentification(infer);
    } 
    else if (infer.name == kInferTpye_FaceReidentification) {
      object = createFaceReidentification(infer);
    } 
    else if (infer.name == kInferTpye_PersonAttribsDetection) {
      object = createPersonAttribsDetection(infer);
    }
    else if (infer.name == kInferTpye_LandmarksDetection) {
      object = createLandmarksDetection(infer);
    }
    else if (infer.name == kInferTpye_VehicleAttribsDetection) {
      object = createVehicleAttribsDetection(infer);
    }
    else if (infer.name == kInferTpye_LicensePlateDetection) {
      object = createLicensePlateDetection(infer);
    } 
    else {
      slog::err << "Invalid inference name: " << infer.name << slog::endl;
    }

    if (object != nullptr) {
      inferences.insert({infer.name, object});
      slog::info << " ... Adding one Inference: " << infer.name << slog::endl;
    }
  }

  return inferences;
}

std::shared_ptr<dynamic_vino_lib::BaseInference>
PipelineManager::createFaceDetection(
    const Params::ParamManager::InferenceRawData& infer) {
  // TODO: add batch size in param_manager
  auto face_detection_model =
      std::make_shared<Models::FaceDetectionModel>(infer.model, 1, 1, infer.batch);
  face_detection_model->modelInit();
  auto face_detection_engine = engine_manager_.createEngine(
    infer.engine, face_detection_model);
  auto face_inference_ptr = std::make_shared<dynamic_vino_lib::FaceDetection>(
      0.5);  // TODO: add output_threshold in param_manager
  face_inference_ptr->loadNetwork(face_detection_model);
  face_inference_ptr->loadEngine(face_detection_engine);

  return face_inference_ptr;
}

std::shared_ptr<dynamic_vino_lib::BaseInference>
PipelineManager::createAgeGenderRecognition(
    const Params::ParamManager::InferenceRawData& param) {
  auto model =
      std::make_shared<Models::AgeGenderDetectionModel>(param.model, 1, 2, param.batch);
  model->modelInit();
  auto engine = engine_manager_.createEngine(param.engine, model);
  auto infer = std::make_shared<dynamic_vino_lib::AgeGenderDetection>();
  infer->loadNetwork(model);
  infer->loadEngine(engine);

  return infer;
}

std::shared_ptr<dynamic_vino_lib::BaseInference>
PipelineManager::createEmotionRecognition(
    const Params::ParamManager::InferenceRawData& param) {
  auto model =
      std::make_shared<Models::EmotionDetectionModel>(param.model, 1, 1, param.batch);
  model->modelInit();
  auto engine = engine_manager_.createEngine(param.engine, model);
  auto infer = std::make_shared<dynamic_vino_lib::EmotionsDetection>();
  infer->loadNetwork(model);
  infer->loadEngine(engine);

  return infer;
}

std::shared_ptr<dynamic_vino_lib::BaseInference>
PipelineManager::createHeadPoseEstimation(
    const Params::ParamManager::InferenceRawData& param) {
  auto model =
      std::make_shared<Models::HeadPoseDetectionModel>(param.model, 1, 3, param.batch);
  model->modelInit();
  auto engine = engine_manager_.createEngine(param.engine, model);
  auto infer = std::make_shared<dynamic_vino_lib::HeadPoseDetection>();
  infer->loadNetwork(model);
  infer->loadEngine(engine);

  return infer;
}

std::shared_ptr<dynamic_vino_lib::BaseInference>
PipelineManager::createObjectDetection(
const Params::ParamManager::InferenceRawData & infer)
{
  std::shared_ptr<Models::ObjectDetectionModel> object_detection_model;
  std::shared_ptr<dynamic_vino_lib::ObjectDetection> object_inference_ptr;

  if (infer.model_type == kInferTpye_ObjectDetectionTypeSSD)
  {
    object_detection_model = 
      std::make_shared<Models::ObjectDetectionSSDModel>(infer.model, 1, 1, infer.batch);

    object_inference_ptr = std::make_shared<dynamic_vino_lib::ObjectDetectionSSD>(
    0.5); // To-do theshold configuration
  }

  if (infer.model_type == kInferTpye_ObjectDetectionTypeYolov2voc)
  {
    object_detection_model = 
      std::make_shared<Models::ObjectDetectionYolov2vocModel>(infer.model, 1, 1, infer.batch);

    object_inference_ptr = std::make_shared<dynamic_vino_lib::ObjectDetectionYolov2voc>(
    0.5); // To-do theshold configuration
  }

  object_detection_model->modelInit();
  auto object_detection_engine = engine_manager_.createEngine(
    infer.engine, object_detection_model);
  object_inference_ptr->loadNetwork(object_detection_model);
  object_inference_ptr->loadEngine(object_detection_engine);

  return object_inference_ptr;
}

std::shared_ptr<dynamic_vino_lib::BaseInference>
PipelineManager::createObjectSegmentation(const Params::ParamManager::InferenceRawData & infer)
{
  auto model =
    std::make_shared<Models::ObjectSegmentationModel>(infer.model, 2, 2, infer.batch);
  model->modelInit();
  slog::info << "Segmentation model initialized." << slog::endl;
  auto engine = engine_manager_.createEngine(infer.engine, model);
  slog::info << "Segmentation Engine initialized." << slog::endl;
  auto segmentation_inference_ptr = std::make_shared<dynamic_vino_lib::ObjectSegmentation>(
    infer.confidence_threshold);
    slog::info << "Segmentation Inference instanced." << slog::endl;
  segmentation_inference_ptr->loadNetwork(model);
  segmentation_inference_ptr->loadEngine(engine);

  return segmentation_inference_ptr;
}

std::shared_ptr<dynamic_vino_lib::BaseInference>
PipelineManager::createPersonReidentification(
  const Params::ParamManager::InferenceRawData & infer)
{
  auto person_reidentification_model =
    std::make_shared<Models::PersonReidentificationModel>(infer.model, 1, 1, infer.batch);
  person_reidentification_model->modelInit();
  auto person_reidentification_engine = engine_manager_.createEngine(
    infer.engine, person_reidentification_model);
  auto reidentification_inference_ptr =
    std::make_shared<dynamic_vino_lib::PersonReidentification>(infer.confidence_threshold);
  reidentification_inference_ptr->loadNetwork(person_reidentification_model);
  reidentification_inference_ptr->loadEngine(person_reidentification_engine);

  return reidentification_inference_ptr;
}

std::shared_ptr<dynamic_vino_lib::BaseInference>
PipelineManager::createFaceReidentification(
  const Params::ParamManager::InferenceRawData & infer)
{
  auto face_reidentification_model =
    std::make_shared<Models::FaceReidentificationModel>(infer.model, 1, 1, infer.batch);
  face_reidentification_model->modelInit();
  auto face_reidentification_engine = engine_manager_.createEngine(
    infer.engine, face_reidentification_model);
  auto face_reid_ptr =
    std::make_shared<dynamic_vino_lib::FaceReidentification>(infer.confidence_threshold);
  face_reid_ptr->loadNetwork(face_reidentification_model);
  face_reid_ptr->loadEngine(face_reidentification_engine);

  return face_reid_ptr;
}

std::shared_ptr<dynamic_vino_lib::BaseInference>
PipelineManager::createPersonAttribsDetection(
  const Params::ParamManager::InferenceRawData & infer)
{
  auto person_attribs_detection_model =
    std::make_shared<Models::PersonAttribsDetectionModel>(infer.model, 1, 1, infer.batch);
  person_attribs_detection_model->modelInit();
  auto person_attribs_detection_engine = engine_manager_.createEngine(
    infer.engine, person_attribs_detection_model);
  auto attribs_inference_ptr =
    std::make_shared<dynamic_vino_lib::PersonAttribsDetection>(infer.confidence_threshold);
  attribs_inference_ptr->loadNetwork(person_attribs_detection_model);
  attribs_inference_ptr->loadEngine(person_attribs_detection_engine);

  return attribs_inference_ptr;
}

std::shared_ptr<dynamic_vino_lib::BaseInference>
PipelineManager::createLandmarksDetection(
  const Params::ParamManager::InferenceRawData & infer)
{
  auto landmarks_detection_model =
    std::make_shared<Models::LandmarksDetectionModel>(infer.model, 1, 1, infer.batch);
  landmarks_detection_model->modelInit();
  auto landmarks_detection_engine = engine_manager_.createEngine(
    infer.engine, landmarks_detection_model);
  auto landmarks_inference_ptr =
    std::make_shared<dynamic_vino_lib::LandmarksDetection>();
  landmarks_inference_ptr->loadNetwork(landmarks_detection_model);
  landmarks_inference_ptr->loadEngine(landmarks_detection_engine);

  return landmarks_inference_ptr;
}




std::shared_ptr<dynamic_vino_lib::BaseInference>
PipelineManager::createVehicleAttribsDetection(
  const Params::ParamManager::InferenceRawData & infer)
{
  auto vehicle_attribs_model =
    std::make_shared<Models::VehicleAttribsDetectionModel>(infer.model, 1, 2, infer.batch);
  vehicle_attribs_model->modelInit();
  auto vehicle_attribs_engine = engine_manager_.createEngine(
    infer.engine, vehicle_attribs_model);
  auto vehicle_attribs_ptr =
    std::make_shared<dynamic_vino_lib::VehicleAttribsDetection>();
  vehicle_attribs_ptr->loadNetwork(vehicle_attribs_model);
  vehicle_attribs_ptr->loadEngine(vehicle_attribs_engine);

  return vehicle_attribs_ptr;
}

std::shared_ptr<dynamic_vino_lib::BaseInference>
PipelineManager::createLicensePlateDetection(
  const Params::ParamManager::InferenceRawData & infer)
{
  auto license_plate_model =
    std::make_shared<Models::LicensePlateDetectionModel>(infer.model, 2, 1, infer.batch);
  license_plate_model->modelInit();
  auto license_plate_engine = engine_manager_.createEngine(
    infer.engine, license_plate_model);
  auto license_plate_ptr =
    std::make_shared<dynamic_vino_lib::LicensePlateDetection>();
  license_plate_ptr->loadNetwork(license_plate_model);
  license_plate_ptr->loadEngine(license_plate_engine);

  return license_plate_ptr;
}

void PipelineManager::threadPipeline(const char* name) {
  PipelineData& p = pipelines_[name];
  while ( p.state != PipelineState_ThreadStopped && p.pipeline != nullptr && ros::ok()) {
    
    if(p.state != PipelineState_ThreadPasued)
    {
      ros::spinOnce();
      p.pipeline->runOnce();
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }
}
void PipelineManager::runAll() {
  for (auto it = pipelines_.begin(); it != pipelines_.end(); ++it) {
    if(it->second.state != PipelineState_ThreadRunning) {
      it->second.state = PipelineState_ThreadRunning;
    }
    if(it->second.thread == nullptr) {
      it->second.thread = std::make_shared<std::thread>(&PipelineManager::threadPipeline, this, it->second.params.name.c_str());
    }
  }

    
}

void PipelineManager::stopAll()
{
  for (auto it = pipelines_.begin(); it != pipelines_.end(); ++it) {
    if (it->second.state == PipelineState_ThreadRunning) {
      it->second.state = PipelineState_ThreadStopped;
    }
  }

}
void PipelineManager::runService() 
{
  auto node = std::make_shared<vino_service::PipelineProcessingServer
            <pipeline_srv_msgs::PipelineSrv>>("pipeline_service");
  ros::spin();//hold the thread waiting for pipeline service

}
void PipelineManager::joinAll() {
 
  auto service_thread = std::make_shared<std::thread>(&PipelineManager::runService,this);
  service_thread->join();// pipeline service

  for (auto it = pipelines_.begin(); it != pipelines_.end(); ++it) {
    if(it->second.thread != nullptr && it->second.state == PipelineState_ThreadRunning) {
      it->second.thread->join();
    }
  }
}