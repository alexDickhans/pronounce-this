#pragma once

#include <string>
#include <unordered_map>
#include "utils/path.hpp"

namespace Pronounce {
	class Config {
	private:
		std::unordered_map<std::string, double> doubleMap;
		std::unordered_map<std::string, int> intMap;
		std::unordered_map<std::string, std::string> stringMap;
		std::unordered_map<std::string, Config> configMap;
		Path path;
	public:
		Config();

		void setDouble(std::string name, double value) {
			doubleMap[name] = value;
		}

		double getDouble(std::string name, double defaultValue) {
			if (doubleMap.find(name) == doubleMap.end()) {
				return defaultValue;
			} else {
				return doubleMap.at(name);
			}
		}

		double getDouble(std::string name) {
			return getDouble(name, 0.0);
		}

		void setInteger(std::string name, int value) {
			intMap[name] = value;
		}

		int getInteger(std::string name, int defaultValue) {
			if (intMap.find(name) == intMap.end()) {
				return defaultValue;
			} else {
				return intMap.at(name);
			}
		}

		int getInteger(std::string name) {
			return this->getInteger(name, 0);
		}

		void setString(std::string name, std::string value) {
			stringMap[name] = value;
		}

		std::string getString(std::string name, std::string defaultValue) {
			if (stringMap.find(name) == stringMap.end()) {
				return defaultValue;
			} else {
				return stringMap.at(name);
			}
		}

		std::string getString(std::string name) {
			return this->getString(name, "");
		}

		void setConfig(std::string name, Config config) {
			this->configMap[name] = config;
		}
		
		Config getConfig(std::string name) {
			if (configMap.find(name) == configMap.end()) {
				return Config();
			} else {
				return configMap.at(name);
			}
		}

		Path getPath() {
			return path;
		}

		void setPath(Path path) {
			this->path = path;
		}

		~Config();
	};

	Config::Config(/* args */)
	{
	}

	Config::~Config()
	{
	}

} // namespace Pronounce
