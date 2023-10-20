#pragma once

#include "api.h"

namespace Pronounce {
	class CatapultHold : public Behavior {
	private:
		pros::Motor_Group& catapultMotors;
		double setpoint;
	public:
		CatapultHold(std::string name, pros::Motor_Group& motors, double setpoint) : Behavior(name), catapultMotors(motors) {
			this->setpoint = setpoint;
		}

		void initialize() override {
			catapultMotors.set_brake_modes(pros::E_MOTOR_BRAKE_HOLD);
			catapultMotors.set_encoder_units(pros::E_MOTOR_ENCODER_ROTATIONS);
			double position = catapultMotors.get_positions().at(0) - fmod(catapultMotors.get_positions().at(0), 3);
			double newSetpoint = position + setpoint;

			while (newSetpoint < catapultMotors.get_positions().at(0)) {
				newSetpoint += 3;
			}
			catapultMotors.move_absolute(newSetpoint, 200);
		}

		void exit() override {
			catapultMotors.set_brake_modes(pros::E_MOTOR_BRAKE_COAST);
		}

		bool isDone() override {
			return false;
		}
	};
}