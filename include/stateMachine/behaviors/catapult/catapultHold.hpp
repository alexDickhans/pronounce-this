#pragma once

#include "api.h"

namespace Pronounce {
	class CatapultHold : public Behavior {
	private:
		pros::Motor_Group& catapultMotors;
		double setpoint;
		PID* pid;
	public:
		CatapultHold(std::string name, pros::Motor_Group& motors, double setpoint, PID* pid) : Behavior(name), catapultMotors(motors), pid(pid) {
			this->setpoint = setpoint;
		}

		void initialize() override {
			catapultMotors.set_encoder_units(pros::E_MOTOR_ENCODER_ROTATIONS);
			double position = catapultMotors.get_positions().at(0) - fmod(catapultMotors.get_positions().at(0), 1.5);
			double newSetpoint = position + setpoint;

			while (newSetpoint < catapultMotors.get_positions().at(0)) {
				newSetpoint += 1.5;
			}

			pid->setTarget(newSetpoint);
		}

		void update() override {
			catapultMotors.move_voltage(pid->update(catapultMotors.get_positions().at(0)) * 1.2e4);
		}

		void exit() override {
		}

		bool isDone() override {
			return false;
		}
	};
}