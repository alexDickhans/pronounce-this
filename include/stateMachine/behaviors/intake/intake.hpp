#pragma once

#include "api.h"
#include "stateMachine/behavior.hpp"
#include "utils/motorGroup.hpp"

// TODO: Add docstrings
// TODO: add comments

namespace Pronounce {
	class Intake : public Behavior {
	private:
		MotorGroup* bottomIntake;
		MotorGroup* topIntake;
		double topSpeed;
		double bottomSpeed;
	public:
		Intake(MotorGroup* bottomIntake, MotorGroup* topIntake, double topSpeed, double bottomSpeed) {
			this->bottomIntake = bottomIntake;
			this->topIntake = topIntake;
			this->bottomSpeed = bottomSpeed;
			this->topSpeed = topSpeed;
		}

		void initialize() {

		}

		void update() {
			bottomIntake->move_velocity(200.0*bottomSpeed);
			topIntake->move_velocity(200.0*topSpeed);
		}

		void exit() {
			bottomIntake->move_velocity(0.0);
			topIntake->move_velocity(0.0);
		}

		bool isDone() {
			return false;
		}

		~Intake() {}
	};	
} // namespace Pronounce
