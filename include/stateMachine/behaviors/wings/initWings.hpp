#pragma once

#include "wings.hpp"
#include "stateMachine/stateController.hpp"
#include "hardware/hardware.hpp"

namespace Pronounce {
	Wing leftWingIn("WingIn", leftSolenoid, false);
	Wing leftWingOut("WingOut", leftSolenoid, true);
	Wing rightWingIn("WingIn", rightSolenoid, false);
	Wing rightWingOut("WingOut", rightSolenoid, true);

	StateController leftWingStateController("WingsStateController", &leftWingIn);
	StateController rightWingStateController("WingsStateController", &rightWingIn);

	void initWings() {

	}
}