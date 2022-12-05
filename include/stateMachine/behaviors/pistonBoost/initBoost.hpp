#pragma once

#include "api.h"
#include "pistonBoost.hpp"
#include "hardware/hardware.hpp"
#include "stateMachine/stateController.hpp"

namespace Pronounce {
	PistonBoost pistonBoostBoosting("PistonBoostBoost", &pistonBoost, &pistonOverfill, BoostState::boost);
	PistonBoost pistonBoostOverfill("PistonBoostOverfill", &pistonBoost, &pistonOverfill, BoostState::overfill);
	PistonBoost pistonBoostNone("PistonBoostNone", &pistonBoost, &pistonOverfill, BoostState::none);

	StateController pistonBoostStateController("PistonBoost", &pistonBoostNone);

	void initBoost() {

	}
} // namespace Pronounce
