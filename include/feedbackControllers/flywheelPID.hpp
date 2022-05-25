#pragma once

#include "feedbackController.hpp"
#include "pid.hpp"

namespace Pronounce {
	class FlywheelPID : public PID {
	private:
		double feedforwardMultiplier;
	public:
		FlywheelPID() {}

		double update(double input) {
			return calculatePidValues(input) + input * feedforwardMultiplier;
		}

		~FlywheelPID() {}
	};
} // namespace Pronounce
