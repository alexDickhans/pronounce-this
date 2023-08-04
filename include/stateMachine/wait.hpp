#pragma once

#include "units/units.hpp"

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
		QTime startTime;
		QTime duration;
		Behavior* behavior;
	public:
		Wait(Behavior* behavior, QTime duration) {
			this->duration = duration;
			this->behavior = behavior;
			this->setName(behavior->getName()+std::to_string(duration.Convert(second))+"s");
		}

		void initialize() {
			this->startTime = get_time_ms()*1_ms;
			behavior->initialize();
		}

		void update() {
			behavior->update();
		}

		bool isDone() {
			return (get_time_ms()*1_ms) - this->startTime >= this->duration;
		}

		void exit() {
			behavior->exit();
		}
	};
} // namespace Pronounce
