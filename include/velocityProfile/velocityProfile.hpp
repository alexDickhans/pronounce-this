#pragma once

#include "units/units.hpp"

namespace Pronounce {
	struct ProfileConstraints {
		QSpeed maxVelocity;
		QAcceleration maxAcceleration;
		QJerk maxJerk;
	};

	class VelocityProfile {
	private:
		QLength distance;

		ProfileConstraints profileConstraints;

		QSpeed initialSpeed = 0.0;
		QSpeed endSpeed = 0.0;
	public:
		VelocityProfile();

		VelocityProfile(QLength distance, ProfileConstraints profileConstraints);

		VelocityProfile(QLength distance, ProfileConstraints profileConstraints, QSpeed initialSpeed, QSpeed endSpeed);

		[[nodiscard]] const QSpeed &getInitialSpeed() const;
		void setInitialSpeed(const QSpeed &initialSpeed);
		[[nodiscard]] const QSpeed &getEndSpeed() const;
		void setEndSpeed(const QSpeed &endSpeed);

		virtual QTime getDuration() { return 0.0; }

		virtual QLength getDistanceByTime(QTime) { return 0.0; }

		virtual QSpeed getVelocityByTime(QTime time) { return 0.0; }

		virtual QAcceleration getAccelerationByTime(QTime time) { return 0.0; }

		virtual QJerk getJerkByTime(QTime time) { return 0.0; }

		const QLength &getDistance() const;
		virtual void setDistance(const QLength &distance);
		const ProfileConstraints &getProfileConstraints() const;
		virtual void setProfileConstraints(const ProfileConstraints &profileConstraints);

		virtual void calculate(int granularity) {}

		~VelocityProfile() {}
	};
} // namespace Pronounce
