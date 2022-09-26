#pragma once

#include "api.h"
#include "utils/ADIDigitalOutGroup.hpp"
#include "stateMachine/behavior.hpp"
#include "stateMachine/stateController.hpp"
#include "stateMachine/wait.hpp"

namespace Pronounce {
	ADIDigitalOutGroup endgameDigitalOutputs;

	Endgame endgameDisabled("EndgameDisabled", endgameDigitalOutputs, false);
	Endgame endgameEnabled("EndgameEnabled", endgameDigitalOutputs, true);

	StateController endgameStateController("EndgameStateController", &endgameEnabled);

	void initEndgame() {
		endgameDigitalOutputs.addDigitalOutput(new pros::ADIDigitalOut(8, false));
		endgameDigitalOutputs.addDigitalOutput(new pros::ADIDigitalOut(7, false));
	}
}