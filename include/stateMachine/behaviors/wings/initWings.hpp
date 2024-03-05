#pragma once

#include "wings.hpp"
#include "stateMachine/stateController.hpp"
#include "hardware/hardware.hpp"

namespace Pronounce {
	auto leftWingIn = std::make_shared<Wing>("WingIn", leftSolenoid, false);
	auto leftWingOut = std::make_shared<Wing>("WingOut", leftSolenoid, true);
	auto rightWingIn = std::make_shared<Wing>("WingIn", rightSolenoid, false);
	auto rightWingOut = std::make_shared<Wing>("WingOut", rightSolenoid, true);
	auto hangIn = std::make_shared<Wing>("WingIn", hangSolenoid, false);
	auto hangOut = std::make_shared<Wing>("WingOut", hangSolenoid, true);
	auto awpIn = std::make_shared<Wing>("WingIn", AWPSolenoid, false);
	auto awpOut = std::make_shared<Wing>("WingOut", AWPSolenoid, true);

	auto leftWingStateController = std::make_shared<StateController>("WingsStateController", leftWingIn);
	auto rightWingStateController = std::make_shared<StateController>("WingsStateController", rightWingIn);
	auto hangStateController = std::make_shared<StateController>("HangReleaseStateController", hangIn);
	auto awpStateController = std::make_shared<StateController>("HangReleaseStateController", awpIn);

	void initWings() {
		Log("Wings Init");
		Log("Wings Init Done");
	}
}