#pragma once

#include <utility>

#include "api.h"
#include "stateMachine/behavior.hpp"

namespace Pronounce {
	class Blocker : public Behavior{
	private:
		pros::Motor& blockerMotor;
		double target = 0.0;
		int32_t speed = 0.0;
	public:
		Blocker(std::string name, pros::Motor& blockerMotor, double target, int32_t speed) : Behavior(std::move(name)), blockerMotor(blockerMotor) {
			this->target = target;
			this->speed = speed;
		}

		void initialize() override {
			blockerMotor.set_encoder_units(pros::E_MOTOR_ENCODER_DEGREES);
			blockerMotor.move_absolute(target, speed);
		}

		void update() override {
			blockerMotor.move_absolute(target, speed);
		}

		void exit() override {

		}

		bool isDone() override {
			return abs(blockerMotor.get_target_position() - blockerMotor.get_position()) < 10.0;
		}
	};
}