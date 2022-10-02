#pragma once

#include "api.h"
#include "utils/ADIDigitalOutGroup.hpp"
#include "stateMachine/behavior.hpp"
#include "stateMachine/stateController.hpp"
#include "stateMachine/wait.hpp"

namespace Pronounce {
	pros::ADIDigitalOut endgameDigitalOutputs(7, false);

	Endgame endgameDisabled("EndgameDisabled", endgameDigitalOutputs, false);
	Endgame endgameEnabled("EndgameEnabled", endgameDigitalOutputs, true);

	StateController endgameStateController("EndgameStateController", &endgameDisabled);

	void initEndgame() {
	}
}