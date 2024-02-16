#include "sinusoidalVelocityProfile.hpp"

namespace Pronounce {
	SinusoidalVelocityProfile::SinusoidalVelocityProfile(QLength distance, Pronounce::ProfileConstraints profileConstraints, QSpeed initalVelocity, QSpeed endVelocity) : VelocityProfile(fabs(distance.getValue()) * 1_m, profileConstraints, initalVelocity, endVelocity) {
		reversed = signnum_c(distance.getValue()) == -1;
	}

	SinusoidalVelocityProfile::SinusoidalVelocityProfile(QLength distance, QSpeed maxVelocity, QAcceleration maxAcceleration, QJerk maxJerk, QSpeed initalSpeed, QSpeed endSpeed)  : VelocityProfile(fabs(distance.getValue()) * 1_m, ProfileConstraints(), initalSpeed.getValue() * signnum_c(distance.getValue()), endSpeed.getValue() * signnum_c(distance.getValue())) {
		ProfileConstraints profileConstraints;
		profileConstraints.maxVelocity = maxVelocity;
		profileConstraints.maxAcceleration = maxAcceleration;
		profileConstraints.maxJerk = maxJerk;

		this->setProfileConstraints(profileConstraints);

		reversed = signnum_c(distance.getValue()) == -1;
	}

	QTime SinusoidalVelocityProfile::getDuration()  {
		return Tt;
	}

	QLength SinusoidalVelocityProfile::getDistanceByTime(QTime t) {
		if (t.getValue() < startTime.getValue()) {
			return (reversed ? -1 : 1) * ((startB*startOmega*t.getValue() + startSlope*sin(startOmega*t.getValue()))/startOmega);
		} else if (t.getValue() <= endStartTime.getValue() && !isSingleSine) {
			return (reversed ? -1 : 1) * ((startDistance + this->getProfileConstraints().maxVelocity * (t-startTime)).getValue());
		} else if (t.getValue() < Tt.getValue() && !isSingleSine) {
			return (reversed ? -1 : 1) * (this->getDistance().getValue() + ((endB*endOmega*(t-endStartTime).getValue() + endSlope*sin(endOmega*(t-endStartTime).getValue()))/endOmega) - endB*endTime.getValue());
		} else {
			return 0.0;
		}
	}

	QSpeed SinusoidalVelocityProfile::getVelocityByTime(QTime t) {
		if (t.getValue() < startTime.getValue()) {
			return (reversed ? -1 : 1) * (cos(startOmega * t.getValue()) * startSlope + startB);
		} else if (t.getValue() < endStartTime.getValue() && !isSingleSine) {
			return (reversed ? -1 : 1) * (this->getProfileConstraints().maxVelocity).getValue();
		} else if (t.getValue() < Tt.getValue() && !isSingleSine) {
			return (reversed ? -1 : 1) * (cos(endOmega * (t - endStartTime).getValue()) * endSlope + endB);
		} else {
			return 0.0;
		}
	}

	QAcceleration SinusoidalVelocityProfile::getAccelerationByTime(QTime t)  {
		if (t.getValue() < startTime.getValue()) {
			return (reversed ? -1 : 1) * (sin(startOmega * t.getValue()) * startSlope * endOmega);
		} else if (t.getValue() < endStartTime.getValue() && !isSingleSine) {
			return 0.0;
		} else if (t.getValue() < Tt.getValue() && !isSingleSine) {
			return (reversed ? -1 : 1) * (-sin(endOmega * (t - endStartTime).getValue()) * endSlope * endOmega);
		} else {
			return 0.0;
		}
	}

	void SinusoidalVelocityProfile::calculate(int granularity) {

		startSlope = (this->getInitialSpeed() - this->getProfileConstraints().maxVelocity).getValue()/2.0;
		startB = (this->getInitialSpeed() + this->getProfileConstraints().maxVelocity).getValue()/2.0;
		startOmega = this->getProfileConstraints().maxAcceleration.getValue()/startSlope;

		startTime = 1_pi/fabs(startOmega);

		endSlope = (this->getProfileConstraints().maxVelocity - this->getEndSpeed()).getValue()/2.0;
		endB = (this->getEndSpeed() + this->getProfileConstraints().maxVelocity).getValue()/2.0;
		endOmega = this->getProfileConstraints().maxAcceleration.getValue()/endSlope;

		endTime = 1_pi/fabs(endOmega);

		startDistance = startB * startTime.getValue();
		endDistance = endB * endTime.getValue();

		middleDuration = (this->getDistance() - (startDistance + endDistance)) / this->getProfileConstraints().maxVelocity;

		if (middleDuration < 0_s) {
			isSingleSine = true;

			if (signnum_c(this->getDistance().getValue()) * 5_in/second < ((this->getInitialSpeed() + this->getEndSpeed())/2.0)) {
				startSlope = (this->getInitialSpeed() - this->getEndSpeed()).getValue()/2.0;
				startB = (this->getInitialSpeed() + this->getEndSpeed()).getValue()/2.0;
				startTime = this->getDistance()/(startB*1_m/second);
				startOmega = 1_pi/startTime.getValue();

				Tt = startTime;
			} else {
				double a = sqrt(this->getDistance().getValue()/(2*1_pi*this->getProfileConstraints().maxAcceleration.getValue()));
				startSlope = - a * this->getProfileConstraints().maxAcceleration.getValue();
				startB = a * this->getProfileConstraints().maxAcceleration.getValue();
				startTime = 2 * 1_pi * a;
				startOmega = 1.0/a;

				Tt = startTime;
			}

			return;
		}

		endStartTime = startTime + middleDuration;

		Tt = endStartTime + endTime;
	}

	void SinusoidalVelocityProfile::setDistance(QLength distance) {
		VelocityProfile::setDistance(abs(distance.getValue()));
		this->reversed = signnum_c(distance.getValue()) == -1;
	}


}