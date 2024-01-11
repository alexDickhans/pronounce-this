#pragma once

#include "api.h"
#include "stateMachine/behavior.hpp"

namespace Pronounce {

	class Catapult : public Behavior {
	private:
		pros::Motor_Group& catapultMotors;
		double speed;
	public:
		Catapult(std::string name, pros::Motor_Group& motors, double speed) : Behavior(name), catapultMotors(motors) {
			this->speed = speed;
		}

		void initialize() override {
			catapultMotors.move_voltage(1.2e4 * speed);
		}

		void update() override {
			catapultMotors.move_voltage(1.2e4 * speed);
		}

		void exit() override {
			catapultMotors.move_voltage(0.0);
		}

		bool isDone() override {
			return false;
		}
	};
} // Pronounce
