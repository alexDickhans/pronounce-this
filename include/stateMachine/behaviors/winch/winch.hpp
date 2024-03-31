#pragma once

#include "stateMachine/behavior.hpp"
#include "pros/abstract_motor.hpp"

namespace Pronounce {
	class Winch : public Behavior {
	private:
		pros::AbstractMotor& motor;
		double target;
	public:
		Winch(pros::AbstractMotor &motor, double target) : motor(motor), target(target), Behavior("Winch" + std::to_string(target)) {}

		void initialize() override {
			motor.set_encoder_units_all(pros::MotorEncoderUnits::rotations);
			motor.move_absolute(target, 600);
		}

		void update() override {
		}

		void exit() override {
			motor.move_voltage(0);
		}

		bool isDone() override {
			return false;
		}

	};
}