#pragma once

#include "units/units.hpp"

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
	public:
		VelocityProfile() : distance(0.0), profileConstraints() {
			
		}

		VelocityProfile(QLength distance, ProfileConstraints profileConstraints) : distance(distance), profileConstraints(profileConstraints) {}

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

		void setDistance(QLength distance) {
			this->distance = distance;
		}

		ProfileConstraints getProfileConstraints() {
			return this->profileConstraints;
		}

		void setProfileConstraints(ProfileConstraints profileConstraints) {
			this->profileConstraints = profileConstraints;
		}

		~VelocityProfile() {}
	};
} // namespace Pronounce
