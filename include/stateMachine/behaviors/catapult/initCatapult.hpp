#pragma once

#include "hardware/hardware.hpp"
#include "catapult.hpp"

namespace Pronounce {
	Catapult catapultFire("CatapultFire", catapultMotors, 1.0);
	CatapultHold catapultHold("CatapultHold", catapultMotors, 1.25 * 20.0/15.0);
	CatapultHold catapultSkills("CatapultSkills", catapultMotors, 48.0 * 3.0);
	CatapultHold catapultHang("CatapultHang", catapultMotors, 0);

	StateController catapultStateController("CatapultStateController", &catapultHold);

	void initCatapult() {

	}
}