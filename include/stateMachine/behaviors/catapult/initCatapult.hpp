#pragma once

#include "hardware/hardware.hpp"
#include "catapult.hpp"

namespace Pronounce {
	Catapult catapultFire("CatapultFire", catapultMotors, 1.0);
	CatapultHold catapultHold("CatapultHold", catapultMotors, 2.0);
	CatapultHold catapultHang("CatapultHang", catapultMotors,0.1);

	StateController catapultStateController("CatapultStateController", &catapultHold);

	void initCatapult() {
		catapultMotors.set_encoder_units(pros::E_MOTOR_ENCODER_ROTATIONS);
		catapultMotors.at(0).set_zero_position(0.0);
		catapultMotors.at(1).set_zero_position(0.0);
	}
}