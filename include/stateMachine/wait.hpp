#pragma once

int get_time_ms();

#ifdef DEBUG
// Simulation includes/time calculation
#include <chrono>

// Get the current time in milliseconds using the chrono API
int get_time_ms() {
	return std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count();
}

#else

#include "stateMachine/behavior.hpp"
// Robot specific includes/time calculation
#include "api.h"

// Get the current time in milliseconds using the pros API
int get_time_ms() {
	return pros::millis();
}

#endif // DEBUG

// TODO: Add docstring
// TODO: Add comments

namespace Pronounce {
	class Wait : public Behavior {
	private:
		int startTime;
		int duration;
		Behavior* behavior;
	public:
		Wait(Behavior* behavior, int duration) {
			this->duration = duration;
			this->behavior = behavior;
		}

		void initialize() {
			this->startTime = get_time_ms();
			behavior->initialize();
		}

		void update() {
			behavior->update();
		}

		bool isDone() {
			return get_time_ms() - this->startTime >= this->duration;
		}

		void exit() {
			behavior->exit();
		}
	};
} // namespace Pronounce
