#pragma once

#include "hardware/hardware.hpp"
#include "catapult.hpp"

namespace Pronounce {
	PID cataPID(7.0, 0.0, 8.0, 0.0);
	Catapult catapultFire("CatapultFire", catapultMotors, 1.0);
	Catapult catapultDejam("CatapultDejam", catapultMotors, 0.0);
	CatapultHold catapultHold("CatapultHold", catapultMotors, 1.33, &cataPID);

	StateController catapultStateController("CatapultStateController", &catapultHold);

	void initCatapult() {
		catapultMotors.set_encoder_units(pros::E_MOTOR_ENCODER_ROTATIONS);
		catapultMotors.at(0).set_zero_position(0.0);
		catapultMotors.at(1).set_zero_position(0.0);
		cataPID.setIntegralBound(0.5);
		cataPID.setMaxIntegral(1.0);
	}
}