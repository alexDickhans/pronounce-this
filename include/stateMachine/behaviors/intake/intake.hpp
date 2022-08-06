#pragma once

#include "api.h"
#include "stateMachine/behavior.hpp"
#include "utils/motorGroup.hpp"

// TODO: Add docstrings
// TODO: add comments

namespace Pronounce {
	class Intake : public Behavior {
	private:
		MotorGroup* intake;
		double speed;
	public:
		Intake(MotorGroup* intake, double speed) {
			this->intake = intake;
			this->speed = speed;
		}

		void initialize() {

		}

		void update() {
			intake->move_velocity(200.0*speed);
		}

		void exit() {
			intake->move_velocity(0.0);
		}

		bool isDone() {
			return false;
		}

		~Intake() {}
	};	
} // namespace Pronounce
