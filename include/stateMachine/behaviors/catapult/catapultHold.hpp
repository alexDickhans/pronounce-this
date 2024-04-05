#pragma once

#include "api.h"

namespace Pronounce {
	class CatapultHold : public Behavior {
	private:
		pros::AbstractMotor& catapultMotors;
		double setpoint;
		PID* pid;
	public:
		CatapultHold(std::string name, pros::AbstractMotor& motors, double setpoint, PID* pid) : Behavior(name), catapultMotors(motors), pid(pid) {
			this->setpoint = setpoint;
		}

		void initialize() override {
			catapultMotors.set_encoder_units(pros::E_MOTOR_ENCODER_ROTATIONS);
			double position = mean(catapultMotors.get_position_all()) - fmod(mean(catapultMotors.get_position_all()), 1.5);
			double newSetpoint = position + setpoint;

			while (newSetpoint < mean(catapultMotors.get_position_all())) {
				newSetpoint += 1.5;
			}

			pid->setTarget(newSetpoint);
		}

		void update() override {
			catapultMotors.move_voltage(pid->update(mean(catapultMotors.get_position_all())) * 1.2e4);
		}

		void exit() override {
			catapultMotors.move_voltage(0.0);
		}

		bool isDone() override {
			return false;
		}
	};
}