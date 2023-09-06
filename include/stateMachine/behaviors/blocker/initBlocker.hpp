#pragma once

#include "hardware/hardware.hpp"

namespace Pronounce {
	Blocker blockerIdle("BlockerIdle", blockerMotor, 0.0, 150);
	Blocker blockerHigh("BlockerHigh", blockerMotor, 470.0, 200);
	Blocker blockerMatchLoad("BlockerMatchload", blockerMotor, 620.0, 200);

	StateController blockerStateController("BlockerStateController", &blockerIdle);

	void initBlocker() {

	}
}