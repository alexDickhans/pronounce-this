#pragma once

#include "velocityProfile.hpp"
#include "units/units.hpp"
#include "utils/linearInterpolator.hpp"
#include <cmath>

namespace Pronounce {
	class SinusoidalVelocityProfile : public VelocityProfile {
	private:
		double Yf;
		double Ys;
		double Yaux;
		double Ya;
		double Vw;
		QTime To;
		QTime Ta;
		double omega;
		double Ks; 
		double Tk;
		QTime Ts;
		QTime Tt;

		LinearInterpolator distanceToVelocity;
		LinearInterpolator distanceToAcceleration;
		LinearInterpolator distanceToJerk;
		LinearInterpolator distanceToTime;
	public:
		SinusoidalVelocityProfile();

		void calculateValues(QTime t) {
			QLength distance = this->getDistanceByTime(t);
			QSpeed speed = this->getVelocityByTime(t);
			QAcceleration acceleration = this->getAccelerationByTime(t);
			QJerk jerk = this->getJerkByTime(t);

			distanceToVelocity.add(distance.getValue(), speed.getValue());
			distanceToAcceleration.add(distance.getValue(), acceleration.getValue());
			distanceToJerk.add(distance.getValue(), jerk.getValue());
			distanceToTime.add(distance.getValue(), t.getValue());
		}



		QLength getDistanceByTime(QTime t) {
			if (t <= 0.0_s) {
				return 0.0;
			} else if (t <= Ta) {
				return ((this->getProfileConstraints().maxAcceleration.getValue()/4) * pow(t.getValue(), 2)) + (Ks * (cos(omega * t.getValue()) - 1));
			} else if (t <= Ts) {
				return Ys + Vw * (t - Ts).getValue();
			} else {
				return Yf - getDistanceByTime(Tt - t).getValue();
			}
		}

		QSpeed getVelocityByTime(QTime t) {
			if (t <= 0.0_s) {
				return 0.0;
			} else if (t <= Ta) {
				return Ks * omega * (omega * t.getValue() - sin(omega * t.getValue()));
			} else if (t <= Ts) {
				return Vw;
			} else {
				return getVelocityByTime(Tt - t);
			}
		}

		QAcceleration getAccelerationByTime(QTime t) {
			if (t <= 0.0_s) {
				return 0.0;
			} else if (t <= Ta) {
				return (this->getProfileConstraints().maxAcceleration/2).getValue() * (1-cos(omega * t.getValue()));
			} else if (t <= Ts) {
				return 0.0;
			} else {
				return -getAccelerationByTime(Tt - t);
			}
		}

		QJerk getJerkByTime(QTime t) {
			if (t <= 0.0_s) {
				return 0.0;
			} else if (t <= Ta) {
				return 0.5 * this->getProfileConstraints().maxAcceleration.getValue() * omega * sin(omega * t.getValue());
			} else if (t <= Ts) {
				return 0.0;
			} else {
				return -getJerkByTime(Tt - t);
			}
		}

		void calculate(int granularity) {
			Yf = fabs(this->getDistance().getValue());
			Ys = Yf / 2.0;
			Yaux = this->getProfileConstraints().maxVelocity.getValue();
			Ya = Ys <= Yaux ? Ys : Yaux;
			Vw = Ys <= Yaux ? sqrt(Ys * this->getProfileConstraints().maxAcceleration.getValue()) : this->getProfileConstraints().maxVelocity.getValue();
			To = Vw/this->getProfileConstraints().maxAcceleration.getValue();
			Ta = 2 * To;
			omega = 2 * 1_pi;
			Ks = (Ta.getValue() * Vw) / (4 * 1_pi);
			Tk = 2 * ((Ys - Ya) / this->getProfileConstraints().maxVelocity.getValue());
			Ts = Ta.getValue() + (Tk/2);
			Tt = 2 * Ts;

			for (int i = 0; i < granularity; i ++) {
				QTime time = 1.0/(double) granularity * (double) i * Tt;
				calculateValues(time);
			}
		}

		~SinusoidalVelocityProfile();
	};
} // namespace Pronounce
