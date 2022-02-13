#pragma once

#include "utils/position.hpp"
#include "position/odomWheel.hpp"
#include "odometry.hpp"
#include "utils/utils.hpp"
#include "api.h"

using namespace Pronounce;

namespace Pronounce {
	class TankOdom : public Odometry {
	private:
		OdomWheel* odomWheel;

		pros::Imu* imu;

		double tuningFactor = 1.0;

	public:
		TankOdom(OdomWheel* odomWheel, pros::Imu* imu);

		double getTuringFactor() {
			return this->tuningFactor;
		}

		void setTuningFactor(double tuningFactor) {
			this->tuningFactor = tuningFactor;
		}

		OdomWheel* getOdomWheel() {
			return this->odomWheel;
		}

		void setOdomWheel(OdomWheel* odomWheel) {
			this->odomWheel = odomWheel;
		}

		void update();

		void reset(Position* position) {
			odomWheel->reset();
			this->imu->reset();
			this->setPosition(position);
			this->setResetPosition(position);
		}

		void reset() {
		}

		~TankOdom();
	};
} // namespace Pronounce
