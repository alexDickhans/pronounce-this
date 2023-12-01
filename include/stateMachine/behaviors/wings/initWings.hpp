#pragma once

#include "wings.hpp"
#include "stateMachine/stateController.hpp"
#include "hardware/hardware.hpp"

namespace Pronounce {
	Wing leftWingIn("WingIn", leftSolenoid, false);
	Wing leftWingOut("WingOut", leftSolenoid, true);
	Wing rightWingIn("WingIn", rightSolenoid, false);
	Wing rightWingOut("WingOut", rightSolenoid, true);
	Wing blockerIn("BlockerIn", blockerSolenoid, false);
	Wing blockerOut("BlockerOut", blockerSolenoid, true);

	StateController leftWingStateController("WingsStateController", &leftWingIn);
	StateController rightWingStateController("WingsStateController", &rightWingIn);
	StateController blockerStateController("WingsStateController", &blockerIn);

	void initWings() {

	}
}