#pragma once

#include "velocityProfile.hpp"
#include "utils/utils.hpp"

namespace Pronounce {
	class TrapezoidalVelocityProfile : public VelocityProfile {
		QTime ta, ts, td;
	public:
		TrapezoidalVelocityProfile(QLength distance, ProfileConstraints profileConstraints, QSpeed initialSpeed = 0.0, QSpeed endSpeed = 0.0);

		QTime getDuration() override;
		QLength getDistanceByTime(QTime time) override;
		QSpeed getVelocityByDistance(QLength distance) override;
		QSpeed getVelocityByTime(QTime time) override;
		QAcceleration getAccelerationByDistance(QLength distance) override;
		QAcceleration getAccelerationByTime(QTime time) override;

		[[deprecated("Jerk doesn't work with trapezoidal mp")]] QJerk getJerkByDistance(QLength distance) override;
		[[deprecated("Jerk doesn't work with trapezoidal mp")]] QJerk getJerkByTime(QTime time) override;

		QTime getTimeByDistance(QLength distance) override;
		void calculate(int granularity) override;
	};
}