#pragma once

#include "velocityProfile.hpp"

// TODO: add docstrings
// TODO: add comments
// TODO: Make sure implemntation is correct
// TODO: Test
// TODO: Finish implementation

namespace Pronounce {
	class TrapezoidalVelocityProfile : public VelocityProfile {
	private:
		
	public:
		TrapezoidalVelocityProfile() : VelocityProfile() {

		}

		QTime getDuration() {
			return ;
		}

		QTime getTimeByDistance(QLength distance) { return 0.0; }

		QLength getDistanceByTime(QTime) { return 0.0; }

		QSpeed getVelocityByDistance(QLength distance) { return 0.0; }

		QSpeed getVelocityByTime(QTime time) { return 0.0; }

		QAcceleration getAccelerationByTime(QTime time) {
			if (time > this->getProfileConstraints().maxVelocity / this->getProfileConstraints().maxAcceleration && this->getDuration() - time > this->getProfileConstraints().maxVelocity / this->getProfileConstraints().maxAcceleration) {
				return 0.0;
			} else {
				return this->getProfileConstraints().maxAcceleration;
			}
		}

		QAcceleration getAccelerationByDistance(QLength distance) { return getAccelerationByTime(getTimeByDistance(distance)); }

		QJerk getJerkByDistance(QLength distance) { return this->getAccelerationByDistance(distance) != 0.0_mps2 ? INFINITY : 0.0; }

		QJerk getJerkByTime(QTime time) { return this->getAccelerationByTime(time) != 0.0_mps2 ? INFINITY : 0.0; }

		~TrapezoidalVelocityProfile() {}
	};
} // namespace Pronounce
