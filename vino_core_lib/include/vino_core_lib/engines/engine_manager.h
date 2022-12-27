// Copyright (c) 2018-2019 Intel Corporation
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

/**
 * @brief A header file with declaration for NetworkEngine class
 * @file engine_manager.h
 */
#ifndef VINO_CORE_LIB__ENGINES__ENGINE_MANAGER_H
#define VINO_CORE_LIB__ENGINES__ENGINE_MANAGER_H

#pragma once

#include "vino_core_lib/models/base_model.h"
#include "vino_core_lib/engines/engine.h"
#include "openvino/openvino.hpp"

namespace Engines
{
/**
 * @class EngineManager
 * @brief This class is used to create and manage OpenVINO engines.
 */
class EngineManager
{
public:
  /**
   * @brief Create OpenVINO instance by given Engine Name and Network.
   * @return The shared pointer of created Engine instance.
   */
  std::shared_ptr<Engine> createEngine(const std::string&, const std::shared_ptr<Models::BaseModel>&);

private:
#if (defined(USE_OLD_E_PLUGIN_API))
  std::map<std::string, InferenceEngine::InferencePlugin> plugins_for_devices_;

  std::unique_ptr<InferenceEngine::InferencePlugin> makePluginByName(const std::string& device_name,
                                                                     const std::string& custom_cpu_library_message,
                                                                     const std::string& custom_cldnn_message,
                                                                     bool performance_message);

  std::shared_ptr<Engine> createEngine_beforeV2019R2(const std::string&, const std::shared_ptr<Models::BaseModel>&);
#endif

  std::shared_ptr<Engine> createEngine_V2022(const std::string&, const std::shared_ptr<Models::BaseModel>&);
};
}  // namespace Engines

#endif  // VINO_CORE_LIB__ENGINES__ENGINE_MANAGER_H
