#pragma once

#include "velocityProfile.hpp"
#include "utils/utils.hpp"
#include <iostream>

namespace Pronounce {
	class TrapezoidalVelocityProfile : public VelocityProfile {
		QTime ta, ts, td;
		QSpeed cruiseSpeed;
		QAcceleration aa, ad;
	public:
		TrapezoidalVelocityProfile(QLength distance, ProfileConstraints profileConstraints, QSpeed initialSpeed = 0.0, QSpeed endSpeed = 0.0);

		QTime getDuration() override;
		QLength getDistanceByTime(QTime time) override;
		QSpeed getVelocityByTime(QTime time) override;
		QAcceleration getAccelerationByTime(QTime time) override;

		[[deprecated("Jerk doesn't work with trapezoidal mp")]] QJerk getJerkByTime(QTime time) override;

		void calculate(int granularity) override;
	};
}