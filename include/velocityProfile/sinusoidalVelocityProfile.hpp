#pragma once

#include "velocityProfile.hpp"
#include "units/units.hpp"
#include "utils/linearInterpolator.hpp"
#include <cmath>
#include <iostream>

// TODO: add docstrings
// TODO: add comments
// TODO: Make sure implemntation is correct
// TODO: Test

namespace Pronounce {
	class SinusoidalVelocityProfile : public VelocityProfile {
	private:
		double omega;
		QTime Tt = 3.0_s;

		bool isSingleSine = false;

		QTime startTime;
		QTime endTime;
		QTime endStartTime;

		QLength startDistance;
		QLength endDistance;

		double startSlope;
		double endSlope;
		double startOmega;
		double endOmega;
		double startB;
		double endB;

		QTime middleDuration;

	public:
		SinusoidalVelocityProfile(QLength distance, ProfileConstraints profileConstraints) : VelocityProfile(distance, profileConstraints) {
			
		}

		SinusoidalVelocityProfile(QLength distance, QSpeed maxVelocity, QAcceleration maxAcceleration, QJerk maxJerk, QSpeed initalSpeed = 0.0, QSpeed endSpeed = 0.0) : VelocityProfile(distance, ProfileConstraints(), initalSpeed, endSpeed) {
			ProfileConstraints profileConstraints;
			profileConstraints.maxVelocity = maxVelocity;
			profileConstraints.maxAcceleration = maxAcceleration;
			profileConstraints.maxJerk = maxJerk;

			this->setProfileConstraints(profileConstraints);

		}

		QTime getDuration() {
			return Tt;
		}

		QLength getDistanceByTime(QTime t) {
			return 0.0;
			// if (t <= 0.0_s) {
			// 	return 0.0;
			// } else if (t <= Ta) {
			// 	return ((this->getProfileConstraints().maxAcceleration.getValue()/4) * pow(t.getValue(), 2)) + (Ks * (cos(omega * t.getValue()) - 1));
			// } else if (t <= Ts) {
			// 	return Ys + Vw * (t - Ts).getValue();
			// } else {
			// 	return Yf - getDistanceByTime(Tt - t).getValue();
			// }
		}

		QSpeed getVelocityByTime(QTime t) {

			// std::cout << "Initial Speed: " << 

			if (t.getValue() < startTime.getValue()) {
				return cos(startOmega * t.getValue()) * startSlope + startB;
			} else if (t.getValue() < endStartTime.getValue()) {
				return this->getProfileConstraints().maxVelocity;
			} else if (t.getValue() < Tt.getValue()) {
				return cos(endOmega * (t - endStartTime).getValue()) * endSlope + endB;
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

		QAcceleration getAccelerationByTime(QTime t) {
			// if (t <= 0.0_s) {
			// 	return 0.0;
			// } else if (t <= Ta) {
			// 	return (this->getProfileConstraints().maxAcceleration/2).getValue() * (1-cos(omega * t.getValue()));
			// } else if (t <= Ts) {
			// 	return 0.0;
			// } else {
			// 	return -getAccelerationByTime(Tt - t);
			// }
		}

		void calculate(int granularity) {
			
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

			endStartTime = startTime + middleDuration;

			Tt = endStartTime + endTime;

			if (middleDuration < 0_s) {
				isSingleSine = true;
				if (this->getInitialSpeed() != 0.0 && this->getEndSpeed() != 0.0) {
					startSlope = (this->getInitialSpeed() - this->getEndSpeed()).getValue()/2.0;
					startB = (this->getInitialSpeed() + this->getEndSpeed()).getValue()/2.0;
					startOmega = this->getProfileConstraints().maxAcceleration.getValue()/startSlope;
					startTime = 1_pi/fabs(startOmega);

					Tt = startTime;
				}
			}
			// std::cout << Tt.getValue() << std::endl;
		}

		~SinusoidalVelocityProfile() {}
	};
} // namespace Pronounce
