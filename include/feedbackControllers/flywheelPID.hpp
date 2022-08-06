#pragma once

#include "feedbackController.hpp"
#include "pid.hpp"

// TODO: Add comments

namespace Pronounce {
	class FlywheelPID : public PID {
	private:
		double feedforwardMultiplier = 0.0;
	public:
		FlywheelPID() {}
		FlywheelPID(double kP, double kI, double kD, double feedforwardMultiplier) : PID(kP, kI, kD) {
			this->feedforwardMultiplier = feedforwardMultiplier;
		}

		double update(double input) {
			return calculatePidValues(input) + input * feedforwardMultiplier;
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
