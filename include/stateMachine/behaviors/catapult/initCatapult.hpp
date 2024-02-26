#pragma once

#include "hardware/hardware.hpp"
#include "catapult.hpp"

namespace Pronounce {
	Catapult catapultFire("CatapultFire", catapultMotors, 1.0);
	Catapult catapultNone("CatapultFire", catapultMotors, 0.0);

	StateController catapultStateController("CatapultStateController", &catapultNone);

	void initCatapult() {
		catapultMotors.set_encoder_units(pros::E_MOTOR_ENCODER_ROTATIONS);
		catapultMotors.at(0).set_zero_position(0.0);
		catapultMotors.at(1).set_zero_position(0.0);
		catapultMotors.set_brake_modes(pros::E_MOTOR_BRAKE_COAST);
	}
}