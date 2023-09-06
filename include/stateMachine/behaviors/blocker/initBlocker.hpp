#pragma once

#include "hardware/hardware.hpp"

namespace Pronounce {
	Blocker blockerIdle("BlockerIdle", blockerMotor, 0.0, 200);
	Blocker blockerHigh("BlockerHigh", blockerMotor, 450.0, 200);
	Blocker blockerMatchLoad("BlockerMatchload", blockerMotor, 500.0, 200);

	StateController blockerStateController("BlockerStateController", &blockerIdle);

	void initBlocker() {

	}
}