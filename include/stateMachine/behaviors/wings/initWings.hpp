#pragma once

#include "wings.hpp"
#include "stateMachine/stateController.hpp"
#include "hardware/hardware.hpp"

namespace Pronounce {
	auto frontLeftWingIn = std::make_shared<Wing>("WingIn", frontLeftSolenoid, false);
	auto frontLeftWingOut = std::make_shared<Wing>("WingOut", frontLeftSolenoid, true);
	auto frontRightWingIn = std::make_shared<Wing>("WingIn", frontRightSolenoid, false);
	auto frontRightWingOut = std::make_shared<Wing>("WingOut", frontRightSolenoid, true);

	auto frontLeftWingStateController = std::make_shared<StateController>("FrontLeftWing", frontLeftWingIn);
	auto frontRightWingStateController = std::make_shared<StateController>("FrontRightWin", frontRightWingIn);

	void initWings() {
		Log("Wings Init");
		Log("Wings Init Done");
	}
}