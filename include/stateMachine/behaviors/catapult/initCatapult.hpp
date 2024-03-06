#pragma once

#include "hardware/hardware.hpp"
#include "catapult.hpp"
#include "catapultHold.hpp"

namespace Pronounce {
	PID cataPID(9.0, 0.0, 12.0, 0.0);
	auto catapultFire = std::make_shared<Catapult>("CatapultFire", catapultMotors, 1.0);
	auto catapultHold = std::make_shared<CatapultHold>("CatapultHold", catapultMotors, 0.9, &cataPID);

	auto catapultStateController = std::make_shared<StateController>("CatapultStateController", catapultHold);

	void initCatapult() {
		Log("Catapult Init");
		catapultMotors.set_encoder_units(pros::E_MOTOR_ENCODER_ROTATIONS);
		catapultMotors.set_zero_position_all(0.0);
		Log("Catapult Init Done");
	}
}