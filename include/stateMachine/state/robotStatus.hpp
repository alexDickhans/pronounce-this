#pragma once

#include "stateMachine/behavior.hpp"
#include "api.h"
#include "hardware/hardware.hpp"
#include "auton.h"

// TODO: Clean up
// TODO: Implement more sensors with classes

namespace Pronounce {

	class RobotStatus: public Behavior {
	private:
	public:

		RobotStatus() {}

		void initialize() {
			return;
		}

		void update() {
			
		}

		void exit() {
			return;
		}

		~RobotStatus() {}
	};
} // namespace Pronounce
