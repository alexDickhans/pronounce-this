
#include "velocityProfile/trapezoidalVelocityProfile.hpp"

Pronounce::TrapezoidalVelocityProfile::TrapezoidalVelocityProfile(QLength distance, Pronounce::ProfileConstraints profileConstraints, QSpeed initialSpeed, QSpeed endSpeed) : VelocityProfile(distance, profileConstraints, initialSpeed, endSpeed) {
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


QSpeed Pronounce::TrapezoidalVelocityProfile::getVelocityByTime(QTime time) {
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

QJerk Pronounce::TrapezoidalVelocityProfile::getJerkByTime(QTime time) {
	return std::numeric_limits<double>::quiet_NaN();
}

void Pronounce::TrapezoidalVelocityProfile::calculate(int granularity) {
	ProfileConstraints profileConstraints = this->getProfileConstraints();
	cruiseSpeed = signnum_c(this->getDistance().getValue()) * profileConstraints.maxVelocity;
	aa = signnum_c((cruiseSpeed - this->getInitialSpeed()).getValue()) * profileConstraints.maxAcceleration;
	ad = signnum_c((this->getEndSpeed() - cruiseSpeed).getValue()) * profileConstraints.maxAcceleration;
	ta = (cruiseSpeed - this->getInitialSpeed()) / aa;
	td = ad.getValue() == 0.0 ? 0.0 : (this->getEndSpeed() - cruiseSpeed) / ad;
	QLength la = 0.5 * Qsq(ta) * aa + this->getInitialSpeed() * ta;
	QLength ld = 0.5 * Qsq(td) * ad + cruiseSpeed * td;

	QLength ls = this->getDistance() - la - ld;

	if (ls.getValue() * signnum_c(cruiseSpeed.getValue()) > 0) {
		ts = ls / cruiseSpeed + ta;
		td = ts + td;
	} else {
		QAcceleration aa2 = signnum_c((this->getEndSpeed() - this->getInitialSpeed()).getValue()) * profileConstraints.maxAcceleration;
		ta = (cruiseSpeed - this->getInitialSpeed()) / aa2;
		la = 0.5 * Qsq(ta) * aa2 + this->getInitialSpeed() * ta;
		if (abs(la.getValue()) > abs(this->getDistance().getValue())) {
			ta = 2 * this->getDistance() / (this->getInitialSpeed() + this->getEndSpeed());
			aa = (this->getEndSpeed() - this->getInitialSpeed())/ta;
			ts = ta;
			td = ta;
		} else {
			ta = (sqrt(2) * Qsqrt(-Qsq(ad) * (2 * ad * this->getDistance() - Qsq(this->getInitialSpeed()) + Qsq(this->getEndSpeed()))) + 2 * ad * this->getInitialSpeed() - 2 * ad * this->getEndSpeed())/(2 * Qsq(ad));
			td = (this->getInitialSpeed() + ta * aa - this->getEndSpeed())/-ad + ta;
			cruiseSpeed = this->getInitialSpeed() + ta * aa;
			ad = -ad;
			ts = ta;
		}
	}
}
