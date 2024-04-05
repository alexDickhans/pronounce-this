#pragma once

#include "velocityProfile.hpp"
#include "utils/utils.hpp"
#include <iostream>

namespace Pronounce {
	class TrapezoidalVelocityProfile : public VelocityProfile {
		QTime ta, ts, td;
		QVelocity cruiseSpeed;
		QAcceleration aa, ad;
	public:
		TrapezoidalVelocityProfile(QLength distance, ProfileConstraints profileConstraints, QVelocity initialSpeed = 0.0, QVelocity endSpeed = 0.0);

		QTime getDuration() override;
		QLength getDistanceByTime(QTime time) override;
		QVelocity getVelocityByTime(QTime time) override;
		QAcceleration getAccelerationByTime(QTime time) override;

		void calculate() override;
	};
}