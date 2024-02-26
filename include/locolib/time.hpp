#pragma once

#include "units/units.hpp"
#include "pros/rtos.h"

namespace Loco {
	class Time {
	public:
		static QTime currentTime() {
			return pros::c::millis() * 1_ms;
		}

		static void wait(QTime time) {
			pros::c::delay(time.Convert(1_ms));
		}
	};
} // Loco
