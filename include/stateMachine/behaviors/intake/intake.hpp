#pragma once

#include "api.h"
#include "stateMachine/behavior.hpp"

namespace Pronounce {
	class Intake : public Behavior {
	private:
		pros::Motor* intake;
		double speed;
	public:
		Intake(pros::Motor* intake, double speed) {
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
