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
#include "vino_core_lib/models/base_model.h"

namespace Models
{
struct ModelFactory
{
	template<typename T>
	struct register_t
	{
		register_t(const std::string& key)
		{
			ModelFactory::get().getMap().emplace(key, [] { return new T(); });
		}

		template<typename... Args>
		register_t(const std::string& key, Args... args)
		{
			ModelFactory::get().getMap().emplace(key, [&] { return new T(args...); });
		}
	};

	static BaseModel* produce(const std::string& key)
	{
		if (getMap().find(key) == getMap().end())
		{
			slog::err << "ModelFactory:Invalid inference key: " << key << ". Please check your model name and type!" << slog::endl;
			return nullptr;
		}

		return getMap()[key]();
	}

	static std::unique_ptr<BaseModel> produce_unique(const std::string& key)
	{
		return std::unique_ptr<BaseModel>(produce(key));
	}

	static std::shared_ptr<BaseModel> produce_shared(const std::string& key)
	{
		return std::shared_ptr<BaseModel>(produce(key));
	}

	static BaseModel* find(const std::string& key)
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
	ModelFactory() {};
	ModelFactory(const ModelFactory&) = delete;
	ModelFactory(ModelFactory&&) = delete;

	static ModelFactory& get()
	{
		static ModelFactory instance;
		return instance;
	}

	static std::map<std::string, std::function<BaseModel*()>>& getMap()
	{
		return get().map_;
	}
	
	std::map<std::string, std::function<BaseModel*()>> map_;
};

#define REG_MODEL_VNAME(T) reg_model_##T##_
#define REG_MODEL(T, key, ...) \
	using namespace Models;    \
	static ModelFactory::register_t<T> REG_MODEL_VNAME(T)(key, ##__VA_ARGS__);
} 
