#pragma once

#include "wings.hpp"
#include "stateMachine/stateController.hpp"
#include "hardware/hardware.hpp"

namespace Pronounce {
	Wings wingsIn("WingsIn", wingsSolenoid, false);
	Wings wingsOut("WingsIn", wingsSolenoid, true);

	StateController wingsStateController("WingsStateController", &wingsIn);

	void initWings() {

	}
}