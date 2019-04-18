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
 * @brief a header file with declaration of Factory class
 * @file factory.cpp
 */

#include <memory>
#include <string>

#include "dynamic_vino_lib/factory.h"
#include "dynamic_vino_lib/inputs/image_input.h"
#include "dynamic_vino_lib/inputs/realsense_camera.h"
#include "dynamic_vino_lib/inputs/realsense_camera_topic.h"
#include "dynamic_vino_lib/inputs/standard_camera.h"
#include "dynamic_vino_lib/inputs/video_input.h"

std::shared_ptr<Input::BaseInputDevice> Factory::makeInputDeviceByName(
    const std::string& input_device_name, const std::string& input_file_path)
{
  if (input_device_name == "RealSenseCamera")
  {
    return std::make_shared<Input::RealSenseCamera>();
  }
  else if (input_device_name == "StandardCamera")
  {
    return std::make_shared<Input::StandardCamera>();
  }
  else if (input_device_name == "RealSenseCameraTopic")
  {
    return std::make_shared<Input::RealSenseCameraTopic>();
  }
  else if (input_device_name == "Video")
  {
    return std::make_shared<Input::Video>(input_file_path);
  }
  else if (input_device_name == "Image")
  {
    return std::make_shared<Input::Image>(input_file_path);
  }
  else
  {
    throw std::logic_error("Unsupported input category");
  }
}

std::shared_ptr<InferenceEngine::InferencePlugin> Factory::makePluginByName(
    const std::string& device_name,
    const std::string& custom_cpu_library_message,  // FLAGS_l
    const std::string& custom_cldnn_message,        // FLAGS_c
    bool performance_message)                       // FLAGS_pc
{                     

    InferenceEngine::InferencePlugin plugin = InferenceEngine::PluginDispatcher({"../../../lib/intel64", ""}).getPluginByDevice(device_name);
    printPluginVersion(plugin, std::cout);

  /** Load extensions for the CPU plugin **/
  if ((device_name.find("CPU") != std::string::npos))
  {
    plugin.AddExtension(std::make_shared<InferenceEngine::Extensions::Cpu::CpuExtensions>());

    if (!custom_cpu_library_message.empty())
    {
      // CPU(MKLDNN) extensions are loaded as a shared library and passed as a
      // pointer to base
      // extension
      auto extension_ptr = InferenceEngine::make_so_pointer<InferenceEngine::IExtension>(
          custom_cpu_library_message);
      plugin.AddExtension(extension_ptr);
    }

  }
  else if (!custom_cldnn_message.empty())
  {
    plugin.SetConfig(
        {{InferenceEngine::PluginConfigParams::KEY_CONFIG_FILE, custom_cldnn_message}});
  }

  if (performance_message)
  {
    plugin.SetConfig(
        {{InferenceEngine::PluginConfigParams::KEY_PERF_COUNT, InferenceEngine::PluginConfigParams::YES}});
  }

  return std::make_shared<InferenceEngine::InferencePlugin>(
      InferenceEngine::InferenceEnginePluginPtr(plugin));

}
