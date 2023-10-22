#pragma once

#include <string>
#include <stdio.h>

namespace PT {
	class Measurement {
	private:
		std::string groupName;
		std::string measurementName;
        bool newUpdate{true};
	protected:
		std::string formatSingle(std::string value);
		void valueUpdated();
		void valueUsed();
	public:
		Measurement();
		Measurement(std::string name);
		Measurement(std::string groupName, std::string measurementName);

		virtual void update() {}

		// virtual std::string to_string() {
		// 	return format("NULL");
		// }

		virtual std::string to_string() { 
			this->valueUsed();
			return formatSingle("NULL"); 
		}

		bool hasUpdated() { return newUpdate; }

		std::string getName();

		void setName(std::string name);

		~Measurement();
	};
} // namespace PT
