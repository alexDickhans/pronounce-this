#pragma once

#include "velocityProfile.hpp"

namespace Pronounce {
	class TrapezoidalVelocityProfile {
	private:
		
	public:
		TrapezoidalVelocityProfile() {

		}

		QTime getDuration() {
			return ;
		}

		QLength getDistanceByTime(QTime) { return 0.0; }

		QSpeed getVelocityByDistance(QLength distance) { return 0.0; }

		QSpeed getVelocityByTime(QTime time) { return 0.0; }

		QAcceleration getAccelerationByDistance(QLength distance) { return 0.0; }

		QAcceleration getAccelerationByTime(QTime time) { return 0.0; }

		QJerk getJerkByDistance(QLength distance) { return 0.0; }

		QJerk getJerkByTime(QTime time) { return 0.0; }

		QTime getTimeByDistance(QLength distance) { return 0.0; }

		~TrapezoidalVelocityProfile() {}
	};
} // namespace Pronounce
