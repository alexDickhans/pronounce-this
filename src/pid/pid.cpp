#include "pid.hpp"

namespace Pronounce {

	PID::PID() {
		this->kP = 0.0;
		this->kI = 0.0;
		this->kD = 0.0;
		this->target = 0.0;
		this->position = 0.0;
	}

	PID::PID(double kP, double kI, double kD, double target, double position) {
		this->kP = kP;
		this->kI = kI;
		this->kD = kD;
		this->target = target;
		this->position = position;
	}

	double PID::update() {
		double error = position - target;
		double derivitive = error - prevError;

		if (abs(error) < integralBound) {
			totalError += error;
		} else {
			totalError = 0;
		}

		totalError = abs(totalError) > maxIntegral ? signum_c(totalError) * maxIntegral : totalError;

		this->power = error * kP + derivitive * kD + totalError * kI;

		return this->power; 
	}

	PID::~PID() {
	}
} // namespace Pronounce

