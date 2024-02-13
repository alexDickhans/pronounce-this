#pragma once

#include "wings.hpp"
#include "stateMachine/stateController.hpp"
#include "hardware/hardware.hpp"

namespace Pronounce {
	Wing leftWingIn("WingIn", leftSolenoid, false);
	Wing leftWingOut("WingOut", leftSolenoid, true);
	Wing rightWingIn("WingIn", rightSolenoid, false);
	Wing rightWingOut("WingOut", rightSolenoid, true);
	Wing hangReleaseIn("WingIn", hangReleaseSolenoid, false);
	Wing hangReleaseOut("WingOut", hangReleaseSolenoid, true);
	Wing awpIn("WingIn", AWPSolenoid, false);
	Wing awpOut("WingOut", AWPSolenoid, true);

	StateController leftWingStateController("WingsStateController", &leftWingIn);
	StateController rightWingStateController("WingsStateController", &rightWingIn);
	StateController hangReleaseStateController("HangReleaseStateController", &hangReleaseIn);
	StateController awpStateController("HangReleaseStateController", &awpIn);

	void initWings() {

	}
}