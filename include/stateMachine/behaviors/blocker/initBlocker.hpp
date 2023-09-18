#pragma once

#include "hardware/hardware.hpp"
#include "blocker.hpp"

namespace Pronounce {
	Blocker blockerIdle("BlockerIdle", blockerMotor, 0.0, 150);
	Blocker blockerHigh("BlockerHigh", blockerMotor, 600.0, 200);
	Blocker blockerMatchLoad("BlockerMatchload", blockerMotor, 1580.0, 200);
	Blocker blockerReleasePreLoad("BlockerReleasePreLoad", blockerMotor, 300.0, 200);

	StateController blockerStateController("BlockerStateController", &blockerIdle);

	void initBlocker() {

	}
}