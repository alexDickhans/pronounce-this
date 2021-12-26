#include "motorGroup.hpp"

namespace Pronounce {
	MotorGroup::MotorGroup() {
		motors = std::vector<pros::Motor*>();
	}

	MotorGroup::MotorGroup(std::vector<pros::Motor*> motors) {
		this->motors = motors;
	}

	MotorGroup::MotorGroup(pros::Motor* motors ...) {
		this->motors = std::vector<pros::Motor*>();
		this->motors.push_back(motors);
		va_list args;
		va_start(args, motors);
		pros::Motor* m = va_arg(args, pros::Motor*);
		while (m != NULL) {
			this->motors.push_back(m);
			m = va_arg(args, pros::Motor*);
		}
		va_end(args);
	}
	
	MotorGroup::~MotorGroup() {
		for (int i = 0; i < motors.size(); i++) {
			delete motors[i];
		}
	}
} // namespace Pronounce
