#pragma once

#include "api.h"
#include "units/units.hpp"
#include "orientation.hpp"

namespace Pronounce {
	class IMU : public Orientation {
	private:
		pros::Imu& imu;
	public:
		IMU();

		void update() {
			Angle imuAngle = toRadians(imu.get_rotation());

			this->setAngle(imuAngle);
		}

		void reset() {
			this->imu.reset();
			this->setAngle(0.0);
		}

		~IMU();
	};
} // namespace Pronounce
