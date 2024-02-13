#pragma once

#include "hardware/hardware.hpp"
#include "catapult.hpp"

namespace Pronounce {
	PID cataPID(18.0, 0.0, 0.0, 0.0);
	Catapult catapultFire("CatapultFire", catapultMotors, 1.0);
	Catapult catapultDejam("CatapultDejam", catapultMotors, -0.4);
	CatapultHold catapultHold("CatapultHold", catapultMotors, 0.9, &cataPID);

	StateController catapultStateController("CatapultStateController", &catapultHold);

	void initCatapult() {
		catapultMotors.set_encoder_units(pros::E_MOTOR_ENCODER_ROTATIONS);
		catapultMotors.at(0).set_zero_position(0.0);
		cataPID.setIntegralBound(0.5);
		cataPID.setMaxIntegral(1.0);
	}
}