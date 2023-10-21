#pragma once

#ifndef SIM
#include "units/units.hpp"
#endif // !SIM

// TODO: add docstrings
// TODO: add comments
// TODO: Make sure implemntation is correct

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
		VelocityProfile() : distance(0.0), profileConstraints() {
			
		}

		VelocityProfile(QLength distance, ProfileConstraints profileConstraints) : distance(distance), profileConstraints(profileConstraints) {}

		VelocityProfile(QLength distance, ProfileConstraints profileConstraints, QSpeed initialSpeed, QSpeed endSpeed) : distance(distance), profileConstraints(profileConstraints), initialSpeed(initialSpeed), endSpeed(endSpeed) {}

		void setInitialSpeed(QSpeed speed) {
			this->initialSpeed = speed;
		}

		void setEndSpeed(QSpeed speed) {
			this->endSpeed = speed;
		}

		QSpeed getInitialSpeed() {
			return initialSpeed;
		}

		QSpeed getEndSpeed() {
			return endSpeed;
		}

		virtual QTime getDuration() { return 0.0; }

		virtual QLength getDistanceByTime(QTime) { return 0.0; }

		virtual QSpeed getVelocityByDistance(QLength distance) { return 0.0; }

		virtual QSpeed getVelocityByTime(QTime time) { return 0.0; }

		virtual QAcceleration getAccelerationByDistance(QLength distance) { return 0.0; }

		virtual QAcceleration getAccelerationByTime(QTime time) { return 0.0; }

		virtual QJerk getJerkByDistance(QLength distance) { return 0.0; }

		virtual QJerk getJerkByTime(QTime time) { return 0.0; }

		virtual QTime getTimeByDistance(QLength distance) { return 0.0; }

		QLength getDistance() {
			return distance;
		}

		virtual void setDistance(QLength distance) {
			this->distance = distance;
		}

		ProfileConstraints getProfileConstraints() {
			return this->profileConstraints;
		}

		void setProfileConstraints(ProfileConstraints profileConstraints) {
			this->profileConstraints = profileConstraints;
		}

		virtual void calculate(int granularity) {}

		~VelocityProfile() {}
	};
} // namespace Pronounce
