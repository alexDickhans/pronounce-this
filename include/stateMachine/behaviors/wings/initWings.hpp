#pragma once

#include "wings.hpp"
#include "stateMachine/stateController.hpp"
#include "hardware/hardware.hpp"

namespace Pronounce {
	auto leftWingIn = std::make_shared<Wing>("WingIn", frontLeftSolenoid, false);
	auto leftWingOut = std::make_shared<Wing>("WingOut", frontLeftSolenoid, true);
	auto rightWingIn = std::make_shared<Wing>("WingIn", frontRightSolenoid, false);
	auto rightWingOut = std::make_shared<Wing>("WingOut", frontRightSolenoid, true);

	auto leftWingStateController = std::make_shared<StateController>("FrontLeftWing", leftWingIn);
	auto rightWingStateController = std::make_shared<StateController>("FrontRightWin", rightWingIn);

	void initWings() {
		Log("Wings Init");
		Log("Wings Init Done");
	}
}