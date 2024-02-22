
#include "velocityProfile/trapezoidalVelocityProfile.hpp"

Pronounce::TrapezoidalVelocityProfile::TrapezoidalVelocityProfile(QLength distance, Pronounce::ProfileConstraints profileConstraints, QSpeed initialSpeed, QSpeed endSpeed) : VelocityProfile(distance, profileConstraints, initialSpeed, endSpeed) {
}

QTime Pronounce::TrapezoidalVelocityProfile::getDuration() {
	return ta + ts + td;
}

QLength Pronounce::TrapezoidalVelocityProfile::getDistanceByTime(QTime time) {
	if (time < ta) {

	} else if (time < ts + ta) {

	} else if (time < ta + ts + td) {

	}
	return VelocityProfile::getDistance();
}


QSpeed Pronounce::TrapezoidalVelocityProfile::getVelocityByTime(QTime time) {
	if (time < ta) {

	} else if (time < ts + ta) {
		return signnum_c(VelocityProfile::getDistance().getValue()) * this->getProfileConstraints().maxVelocity;
	} else if (time < ta + ts + td) {

	}
	return 0.0;
}

QAcceleration Pronounce::TrapezoidalVelocityProfile::getAccelerationByTime(QTime time) {
	if (time < ta) {

	} else if (time < ts + ta) {
		return 0.0;
	} else if (time < ta + ts + td) {

	}
	return 0.0;
}

QJerk Pronounce::TrapezoidalVelocityProfile::getJerkByTime(QTime time) {
	return std::numeric_limits<double>::quiet_NaN();
}

QTime Pronounce::TrapezoidalVelocityProfile::getTimeByDistance(QLength distance) {
	return VelocityProfile::getTimeByDistance(distance);
}

QJerk Pronounce::TrapezoidalVelocityProfile::getJerkByDistance(QLength distance) {
	return std::numeric_limits<double>::quiet_NaN();
}
QSpeed Pronounce::TrapezoidalVelocityProfile::getVelocityByDistance(QLength distance) {
	return this->getVelocityByTime(this->getTimeByDistance(distance));
}

QAcceleration Pronounce::TrapezoidalVelocityProfile::getAccelerationByDistance(QLength distance) {
	return this->getAccelerationByTime(this->getTimeByDistance(distance));
}

void Pronounce::TrapezoidalVelocityProfile::calculate(int granularity) {
	VelocityProfile::calculate(granularity);
}
