#pragma once

#include "api.h"
#include <vector>

namespace Pronounce {
	class ADIDigitalOutGroup {
	private:
		std::vector<pros::ADIDigitalOut*> digitalOutputs;
	public:
		ADIDigitalOutGroup();

		void addDigitalOutput(pros::ADIDigitalOut* digitalOutput);

		void set_value(std::int32_t value);

		~ADIDigitalOutGroup();
	};
} // namespace Pronounce
