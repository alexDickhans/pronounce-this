#pragma once

#include <string>
#include "config.hpp"

namespace Pronounce {
	class Behavior
	{
	private:
		std::string name;
		Config* config;
		Config* globalConfig;
	public:
		Behavior();
		Behavior(std::string name);
		Behavior(std::string name, Config* config, Config* globalConfig);

		virtual void initialize(Config* stateConfig) {}

		virtual void update() {}

		virtual bool isDone() {}

		virtual void dispose() {}

		std::string getName() {
			return this->name;
		}

		void setName(std::string name) {
			this->name = name;
		}

		Config* getConfig() {
			return config;
		}

		void setConfig(Config* config) {
			this->config = config;
		}

		Config* getGlobalConfig() {
			return globalConfig;
		}

		void setGlobalConfig(Config* globalConfig) {
			this->globalConfig = globalConfig;
		}

		~Behavior();
	};
	
	Behavior::Behavior(/* args */)
	{
	}
	
	Behavior::~Behavior()
	{
	}
	
} // namespace Pronounce
