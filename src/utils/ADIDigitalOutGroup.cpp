#include "ADIDigitalOutGroup.hpp"

namespace Pronounce {
	ADIDigitalOutGroup::ADIDigitalOutGroup() {
	}

	void ADIDigitalOutGroup::addDigitalOutput(pros::ADIDigitalOut* digitalOutput) {
		this->digitalOutputs.emplace_back(digitalOutput);
	}

	void ADIDigitalOutGroup::set_value(std::int32_t value) {
		for (int i = 0; i < digitalOutputs.size(); i++) {
			this->digitalOutputs.at(i)->set_value(value);
		}
	}

	ADIDigitalOutGroup::~ADIDigitalOutGroup() {
	}
} // namespace Pronounce
