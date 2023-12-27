#pragma once

#include "velocityProfile.hpp"
#include "units/units.hpp"
#include "utils/linearInterpolator.hpp"
#include <cmath>
#include <iostream>

namespace Pronounce {
	class SinusoidalVelocityProfile : public VelocityProfile {
	private:
		QTime Tt = 3.0_s;

		bool isSingleSine = false;

		QTime startTime;
		QTime endTime;
		QTime endStartTime;

		QLength startDistance;
		QLength endDistance;

		double startSlope{};
		double endSlope{};
		double startOmega{};
		double endOmega{};
		double startB{};
		double endB{};

		QTime middleDuration;

		bool reversed;

	public:
		SinusoidalVelocityProfile(QLength distance, ProfileConstraints profileConstraints, QSpeed initalVelocity = 0.0, QSpeed endVelocity = 0.0) : VelocityProfile(fabs(distance.getValue()) * 1_m, profileConstraints, initalVelocity, endVelocity) {
			
			reversed = signnum_c(distance.getValue()) == -1;
		}

		SinusoidalVelocityProfile(QLength distance, QSpeed maxVelocity, QAcceleration maxAcceleration, QJerk maxJerk, QSpeed initalSpeed = 0.0, QSpeed endSpeed = 0.0) : VelocityProfile(fabs(distance.getValue()) * 1_m, ProfileConstraints(), initalSpeed.getValue() * signnum_c(distance.getValue()), endSpeed.getValue() * signnum_c(distance.getValue())) {
			ProfileConstraints profileConstraints;
			profileConstraints.maxVelocity = maxVelocity;
			profileConstraints.maxAcceleration = maxAcceleration;
			profileConstraints.maxJerk = maxJerk;

			this->setProfileConstraints(profileConstraints);

			reversed = signnum_c(distance.getValue()) == -1;
		}

		QTime getDuration() override {
			return Tt;
		}

		QLength getDistanceByTime(QTime t) override {
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

		QSpeed getVelocityByTime(QTime t) override {

			if (t.getValue() < startTime.getValue()) {
				return (reversed ? -1 : 1) * (cos(startOmega * t.getValue()) * startSlope + startB);
			} else if (t.getValue() < endStartTime.getValue() && !isSingleSine) {
				return (reversed ? -1 : 1) * (this->getProfileConstraints().maxVelocity).getValue();
			} else if (t.getValue() < Tt.getValue() && !isSingleSine) {
				return (reversed ? -1 : 1) * (cos(endOmega * (t - endStartTime).getValue()) * endSlope + endB);
			} else {
				return 0.0;
			}

			// if (t <= 0.0_s) {
			// 	return 0.0;
			// } else if (t <= Ta) {
			// 	return signnum_c(this->getDistance().getValue()) * Ks * omega * (omega * t.getValue() - sin(omega * t.getValue()));
			// } else if (t <= Ts) {
			// 	return signnum_c(this->getDistance().getValue()) * Vw;
			// } else {
			// 	return getVelocityByTime(Tt - t);
			// }
		}

		 QAcceleration getAccelerationByTime(QTime t) override {
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

		void calculate(int granularity) override {
			
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

					std::cout << "HI time1" << Tt.getValue() << std::endl;
				} else {
					double a = sqrt(this->getDistance().getValue()/(2*1_pi*this->getProfileConstraints().maxAcceleration.getValue()));
					startSlope = - a * this->getProfileConstraints().maxAcceleration.getValue();
					startB = a * this->getProfileConstraints().maxAcceleration.getValue();
					startTime = 2 * 1_pi * a;
					startOmega = 1.0/a;

					Tt = startTime;

					std::cout << "HI time2" << Tt.getValue() << std::endl;
				}

				return;
			}

			endStartTime = startTime + middleDuration;

			Tt = endStartTime + endTime;

			std::cout << "HI time3" << Tt.getValue() << std::endl;
		}

		void setDistance(QLength distance) {
			VelocityProfile::setDistance(abs(distance.getValue()));
			this->reversed = signnum_c(distance.getValue()) == -1;
		}

		~SinusoidalVelocityProfile() {}
	};
} // namespace Pronounce
