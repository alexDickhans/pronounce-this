#include "pid.hpp"

namespace Pronounce {

	PID::PID(double kP = 0, double kI = 0, double kD = 0, double target = 0, double position = 0) {
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

