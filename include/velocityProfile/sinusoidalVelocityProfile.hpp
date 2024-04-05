#pragma once

#include "velocityProfile.hpp"
#include "units/units.hpp"
#include "utils/utils.hpp"
#include <cmath>
#include <iostream>

namespace Pronounce {
	class [[deprecated("Use trap mp")]] SinusoidalVelocityProfile : public VelocityProfile {
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
		SinusoidalVelocityProfile(QLength distance, ProfileConstraints profileConstraints, QVelocity initalVelocity = 0.0, QVelocity endVelocity = 0.0);

		SinusoidalVelocityProfile(QLength distance, QVelocity maxVelocity, QAcceleration maxAcceleration, QJerk maxJerk, QVelocity initalSpeed = 0.0, QVelocity endSpeed = 0.0);

		QTime getDuration() final;

		QLength getDistanceByTime(QTime t) final;

		QVelocity getVelocityByTime(QTime t) final;

		QAcceleration getAccelerationByTime(QTime t) final;

		void calculate() final;

		void setDistance(const QLength& distance) final;

		~SinusoidalVelocityProfile() = default;
	};
} // namespace Pronounce
