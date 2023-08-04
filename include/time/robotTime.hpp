#pragma once

#include "units/units.hpp"
#include "api.h"
#include "time.hpp"

namespace Pronounce {
	QTime currentTime() {
		return pros::millis() * 1_ms;
	}
} // namespace Pronounce
