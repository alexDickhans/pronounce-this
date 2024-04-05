
#include "velocityProfile/trapezoidalVelocityProfile.hpp"

Pronounce::TrapezoidalVelocityProfile::TrapezoidalVelocityProfile(QLength distance, Pronounce::ProfileConstraints profileConstraints, QVelocity initialSpeed, QVelocity endSpeed) : VelocityProfile(distance, profileConstraints, initialSpeed, endSpeed) {
}

QTime Pronounce::TrapezoidalVelocityProfile::getDuration() {
	return td;
}

QLength Pronounce::TrapezoidalVelocityProfile::getDistanceByTime(QTime time) {
	if (time <= ta) {
		return 0.5 * Qsq(time) * aa + this->getInitialSpeed() * time;
	} else if (time <= ts) {
		return 0.5 * Qsq(ta) * aa + this->getInitialSpeed() * ta + cruiseSpeed * (time - ta);
	} else if (time <= td) {
		return getDistanceByTime(ts) + 0.5 * Qsq(time - ts) * ad + (time - ts) * cruiseSpeed;
	}

	return this->getDistance();
}


QVelocity Pronounce::TrapezoidalVelocityProfile::getVelocityByTime(QTime time) {
	if (time <= ta) {
		return time * aa + this->getInitialSpeed();
	} else if (time <= ts) {
		return cruiseSpeed;
	} else if (time <= td) {
		return (time - ts) * ad + cruiseSpeed;
	}
	return 0.0;
}

QAcceleration Pronounce::TrapezoidalVelocityProfile::getAccelerationByTime(QTime time) {
	if (time <= ta) {
		return aa;
	} else if (time <= ts) {
		return 0.0;
	} else if (time <= td) {
		return ad;
	}
	return 0.0;
}

void Pronounce::TrapezoidalVelocityProfile::calculate() {
	ProfileConstraints profileConstraints = this->getProfileConstraints();
	cruiseSpeed = signnum_c(this->getDistance().getValue()) * profileConstraints.maxVelocity;
	aa = signnum_c((cruiseSpeed - this->getInitialSpeed()).getValue()) * profileConstraints.maxAcceleration;
	ad = signnum_c((this->getEndSpeed() - cruiseSpeed).getValue()) * profileConstraints.maxAcceleration;
	ta = aa.getValue() == 0.0 ? 0.0 : (cruiseSpeed - this->getInitialSpeed()) / aa;
	td = ad.getValue() == 0.0 ? 0.0 : (this->getEndSpeed() - cruiseSpeed) / ad;
	QLength la = 0.5 * Qsq(ta) * aa + this->getInitialSpeed() * ta;
	QLength ld = 0.5 * Qsq(td) * ad + cruiseSpeed * td;

	QLength ls = this->getDistance() - la - ld;

	if (ls.getValue() * signnum_c(cruiseSpeed.getValue()) > 0) {
		ts = ls / cruiseSpeed + ta;
		td = ts + td;
	} else {
		QAcceleration aa2 = profileConstraints.maxAcceleration;
		ta = Qabs((this->getEndSpeed() - this->getInitialSpeed())) / aa2;
		la = 0.5 * Qsq(ta) * aa2 + this->getInitialSpeed() * ta;
		if (abs(la.getValue()) > abs(this->getDistance().getValue())) {
			ta = 2 * this->getDistance() / (this->getInitialSpeed() + this->getEndSpeed());
			aa = (this->getEndSpeed() - this->getInitialSpeed())/ta;
			ts = ta;
			td = ta;
		} else {
			ta = (sqrt(2.0) *
					Qsqrt(2 * Qabs(aa) * Qabs(this->getDistance()) + Qsq(this->getInitialSpeed()) + Qsq(this->getEndSpeed())) - 2 * this->getInitialSpeed())/
					(2 * Qabs(aa));
			ts = ta;
			td = 2.0 * ta + ((this->getInitialSpeed() - this->getEndSpeed())/aa);
			cruiseSpeed = this->getInitialSpeed() + ta * aa;
		}
	}
}
