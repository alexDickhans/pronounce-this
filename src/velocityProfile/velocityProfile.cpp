#include "velocityProfile.hpp"

namespace Pronounce {

	VelocityProfile::VelocityProfile() : distance(0.0), profileConstraints() {

	}

	VelocityProfile::VelocityProfile(QLength distance, ProfileConstraints profileConstraints)  : distance(distance), profileConstraints(profileConstraints) {}

	VelocityProfile::VelocityProfile(QLength distance, ProfileConstraints profileConstraints, QVelocity initialSpeed, QVelocity endSpeed)  : distance(distance), profileConstraints(profileConstraints), initialSpeed(initialSpeed), endSpeed(endSpeed) {}

	const QVelocity &VelocityProfile::getInitialSpeed() const {
		return initialSpeed;
	}
	void VelocityProfile::setInitialSpeed(const QVelocity &initialSpeed) {
		VelocityProfile::initialSpeed = initialSpeed;
	}
	const QVelocity &VelocityProfile::getEndSpeed() const {
		return endSpeed;
	}
	void VelocityProfile::setEndSpeed(const QVelocity &endSpeed) {
		VelocityProfile::endSpeed = endSpeed;
	}
	const QLength &VelocityProfile::getDistance() const {
		return distance;
	}
	void VelocityProfile::setDistance(const QLength &distance) {
		VelocityProfile::distance = distance;
	}
	const ProfileConstraints &VelocityProfile::getProfileConstraints() const {
		return profileConstraints;
	}
	void VelocityProfile::setProfileConstraints(const ProfileConstraints &profileConstraints) {
		VelocityProfile::profileConstraints = profileConstraints;
	}


}