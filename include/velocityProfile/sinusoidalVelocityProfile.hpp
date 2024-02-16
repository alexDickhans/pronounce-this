#pragma once

#include "velocityProfile.hpp"
#include "units/units.hpp"
#include "utils/linearInterpolator.hpp"
#include <cmath>
#include <iostream>

namespace Pronounce {
	class SinusoidalVelocityProfile : public VelocityProfile {
	private:
		QTime Tt = 3.0_s;

		bool isSingleSine = false;

		QTime startTime;
		QTime endTime;
		QTime endStartTime;

		QLength startDistance;
		QLength endDistance;

		double startSlope{};
		double endSlope{};
		double startOmega{};
		double endOmega{};
		double startB{};
		double endB{};

		QTime middleDuration;

		bool reversed;

	public:
		SinusoidalVelocityProfile(QLength distance, ProfileConstraints profileConstraints, QSpeed initalVelocity = 0.0, QSpeed endVelocity = 0.0);

		SinusoidalVelocityProfile(QLength distance, QSpeed maxVelocity, QAcceleration maxAcceleration, QJerk maxJerk, QSpeed initalSpeed = 0.0, QSpeed endSpeed = 0.0);

		QTime getDuration() final;

		QLength getDistanceByTime(QTime t) final;

		QSpeed getVelocityByTime(QTime t) final;

		 QAcceleration getAccelerationByTime(QTime t) final;

		void calculate(int granularity) final;

		void setDistance(QLength distance) final;

		~SinusoidalVelocityProfile() = default;
	};
} // namespace Pronounce
