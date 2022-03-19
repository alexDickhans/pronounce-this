#include "pid.hpp"

namespace Pronounce {

	PID::PID() {
		this->kP = 0.0;
		this->kI = 0.0;
		this->kD = 0.0;
		this->target = 0.0;
		this->position = 0.0;
	}

	PID::PID(double kP, double kI, double kD, double target, double position, bool turnPid) {
		this->kP = kP;
		this->kI = kI;
		this->kD = kD;
		this->target = target;
		this->position = position;
		this->turnPid = turnPid;
	}

	double PID::update() {
		if (turnPid) {
			this->error = angleDifference(target, position);
		}
		else {
			this->error = target - position;
		}

		this->derivitive = error - prevError;

		if (abs(error) < integralBound) {
			totalError += error;
		}
		else {
			totalError = 0;
		}

		totalError = abs(totalError) > maxIntegral ? signnum_c(totalError) * maxIntegral : totalError;

		this->power = error * kP + derivitive * kD + totalError * kI;

		prevError = error;

		return this->power;
	}

	PID::~PID() {
	}
} // namespace Pronounce

