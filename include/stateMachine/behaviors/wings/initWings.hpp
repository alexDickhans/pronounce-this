#pragma once

#include "wings.hpp"
#include "stateMachine/stateController.hpp"
#include "hardware/hardware.hpp"

namespace Pronounce {
	Wing leftWingIn("WingIn", leftSolenoid, false);
	Wing leftWingOut("WingOut", leftSolenoid, true);
	Wing rightWingIn("WingIn", rightSolenoid, false);
	Wing rightWingOut("WingOut", rightSolenoid, true);
	Wing hangIn("WingIn", hangSolenoid, false);
	Wing hangOut("WingOut", hangSolenoid, true);
	Wing awpIn("WingIn", AWPSolenoid, false);
	Wing awpOut("WingOut", AWPSolenoid, true);

	StateController leftWingStateController("WingsStateController", &leftWingIn);
	StateController rightWingStateController("WingsStateController", &rightWingIn);
	StateController hangReleaseStateController("HangReleaseStateController", &hangIn);
	StateController awpStateController("HangReleaseStateController", &awpIn);

	void initWings() {

	}
}