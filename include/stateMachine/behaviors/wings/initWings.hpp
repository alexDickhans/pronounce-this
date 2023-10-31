#pragma once

#include "wings.hpp"
#include "stateMachine/stateController.hpp"
#include "hardware/hardware.hpp"

namespace Pronounce {
	Wings wingsIn("WingsIn", leftSolenoid, rightSolenoid, false, false);
	Wings wingsOut("WingsOut", leftSolenoid, rightSolenoid, true, true);
	Wings wingsLeft("WingsLeft", leftSolenoid, rightSolenoid, true, false);
	Wings wingsRight("WingsRight", leftSolenoid, rightSolenoid, false, true);

	StateController wingsStateController("WingsStateController", &wingsIn);

	void initWings() {

	}
}