#pragma once

#include "units/units.hpp"

namespace Pronounce {
	struct ProfileConstraints {
		QVelocity maxVelocity;
		QAcceleration maxAcceleration;
		QJerk maxJerk;
	};

	class VelocityProfile {
	private:
		QLength distance;

		ProfileConstraints profileConstraints;

		QVelocity initialSpeed = 0.0, endSpeed = 0.0;
	public:
		VelocityProfile();

		VelocityProfile(QLength distance, ProfileConstraints profileConstraints);

		VelocityProfile(QLength distance, ProfileConstraints profileConstraints, QVelocity initialSpeed, QVelocity endSpeed);

		[[nodiscard]] const QVelocity &getInitialSpeed() const;
		void setInitialSpeed(const QVelocity &initialSpeed);
		[[nodiscard]] const QVelocity &getEndSpeed() const;
		void setEndSpeed(const QVelocity &endSpeed);

		virtual QTime getDuration() { return 0.0; }

		virtual QLength getDistanceByTime(QTime) { return 0.0; }

		virtual QVelocity getVelocityByTime(QTime time) { return 0.0; }

		virtual QAcceleration getAccelerationByTime(QTime time) { return 0.0; }

		virtual QJerk getJerkByTime(QTime time) { return 0.0; }

		const QLength &getDistance() const;
		virtual void setDistance(const QLength &distance);
		const ProfileConstraints &getProfileConstraints() const;
		virtual void setProfileConstraints(const ProfileConstraints &profileConstraints);

		virtual void calculate() {}

		~VelocityProfile() {}
	};
} // namespace Pronounce
