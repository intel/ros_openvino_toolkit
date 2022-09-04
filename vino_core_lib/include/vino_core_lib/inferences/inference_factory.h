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
#pragma once

#include <map>
#include <string>
#include <functional>
#include <memory>
#include "vino_core_lib/inferences/base_inference.h"

namespace vino_core_lib
{
struct InferenceFactory
{
	template<typename T>
	struct register_t
	{
		register_t(const std::string& key)
		{
			InferenceFactory::get().getMap().emplace(key, [] { return new T(); });
		}

		template<typename... Args>
		register_t(const std::string& key, Args... args)
		{
			InferenceFactory::get().getMap().emplace(key, [&] { return new T(args...); });
		}
	};

	static BaseInference* produce(const std::string& key)
	{
		if (getMap().find(key) == getMap().end())
		{
			slog::err << "Invalid inference name: " << key << slog::endl;
			return nullptr;
		}



		return getMap()[key]();
	}

	static std::unique_ptr<BaseInference> produce_unique(const std::string& key)
	{
		return std::unique_ptr<BaseInference>(produce(key));
	}

	static std::shared_ptr<BaseInference> produce_shared(const std::string& key)
	{
		return std::shared_ptr<BaseInference>(produce(key));
	}

	static BaseInference* find(const std::string& key)
	{
		auto &pf = get();
		auto item = pf.getMap().find(key);

		if(item == pf.getMap().end())
		{
			return nullptr;
		}

		return item->second();
	}

private:
	InferenceFactory() {};
	InferenceFactory(const InferenceFactory&) = delete;
	InferenceFactory(InferenceFactory&&) = delete;

	static InferenceFactory& get()
	{
		static InferenceFactory instance;
		return instance;
	}

	static std::map<std::string, std::function<BaseInference*()>>& getMap()
	{
		return get().map_;
	}
	
	std::map<std::string, std::function<BaseInference*()>> map_;
};

#define REG_INFERENCE_VNAME(T) reg_inference_##T##_
#define REG_INFERENCE(T, key, ...) static InferenceFactory::register_t<T> REG_INFERENCE_VNAME(T)(key, ##__VA_ARGS__);
} 
