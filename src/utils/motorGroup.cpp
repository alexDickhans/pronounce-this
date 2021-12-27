#include "motorGroup.hpp"

namespace Pronounce {
	MotorGroup::MotorGroup() {
		motors = std::vector<pros::Motor*>();
	}

	MotorGroup::MotorGroup(std::vector<pros::Motor*> motors) {
		this->motors = motors;
	}
	
	MotorGroup::~MotorGroup() {
		
	}
} // namespace Pronounce
