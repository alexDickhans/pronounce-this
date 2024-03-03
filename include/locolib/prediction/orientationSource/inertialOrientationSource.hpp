#pragma once

#include "orientationSource.hpp"
#include "pros/imu.h"

namespace Loco {
	class InertialOrientationSource : public OrientationSource {
	private:
		pros::Imu& imu;
	public:
		explicit InertialOrientationSource(pros::Imu& imu) : imu(imu) {

		}

		Angle getAngle() override {
			return -imu.get_rotation() * 1_deg;
		}
	};
}