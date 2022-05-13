#pragma once

#include <string>
#include <unordered_map>

namespace Pronounce
{
	class Config
	{
	private:
		std::unordered_map<std::string, double> doubleMap;
		std::unordered_map<std::string, int> doubleInt;
		std::unordered_map<std::string, std::string> doubleString;

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

		void setInteger(int value) {
			// set the value
		}

		int getInteger() {
			// Return the value
			return 0;
		}

		void setString(std::string value) {
			// set the value
		}

		std::string getString() {
			// return the value
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
