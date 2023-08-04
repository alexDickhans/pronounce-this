#pragma once

#include "feedbackController.hpp"
#include "pid.hpp"
#include <stdio.h>

// TODO: Add comments

namespace Pronounce {
	class FlywheelPID : public PID {
	private:
		double feedforwardMultiplier = 3.6;
	public:
		FlywheelPID() {}
		FlywheelPID(double kP, double kI, double kD, double feedforwardMultiplier) : PID(kP, kI, kD) {
			this->feedforwardMultiplier = feedforwardMultiplier;
		}

		double update(double input) {
			return (this->getTarget() * feedforwardMultiplier) + calculatePidValues(input);
		}

		double getFeedforwardMultiplier() {
			return feedforwardMultiplier;
		}

		void setFeedforwardMultiplier(double feedforwardMultiplier) {
			this->feedforwardMultiplier = feedforwardMultiplier;
		}

		~FlywheelPID() {}
	};
} // namespace Pronounce
