#pragma once

#include "wings.hpp"
#include "stateMachine/stateController.hpp"
#include "hardware/hardware.hpp"

namespace Pronounce {
	auto frontLeftWingIn = std::make_shared<Wing>("WingIn", frontLeftSolenoid, false);
	auto frontLeftWingOut = std::make_shared<Wing>("WingOut", frontLeftSolenoid, true);
	auto frontRightWingIn = std::make_shared<Wing>("WingIn", frontRightSolenoid, false);
	auto frontRightWingOut = std::make_shared<Wing>("WingOut", frontRightSolenoid, true);
	auto backLeftWingIn = std::make_shared<Wing>("WingIn", backLeftSolenoid, false);
	auto backLeftWingOut = std::make_shared<Wing>("WingOut", backLeftSolenoid, true);
	auto backRightWingIn = std::make_shared<Wing>("WingIn", backRightSolenoid, false);
	auto backRightWingOut = std::make_shared<Wing>("WingOut", backRightSolenoid, true);

	auto frontLeftWingStateController = std::make_shared<StateController>("FrontLeftWing", frontLeftWingIn);
	auto frontRightWingStateController = std::make_shared<StateController>("FrontRightWin", frontRightWingIn);
	auto backLeftWingStateController = std::make_shared<StateController>("BackLeftWing", backLeftWingIn);
	auto backRightWingStateController = std::make_shared<StateController>("BackRightWing", backRightWingIn);

	void initWings() {
		Log("Wings Init");
		Log("Wings Init Done");
	}
}