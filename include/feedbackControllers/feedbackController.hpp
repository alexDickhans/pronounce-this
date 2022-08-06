#pragma once

// TODO: Add comments

namespace Pronounce {
	class FeedbackController {
	private:
	protected:
		double target = 0.0;
		double position = 0.0;
		double power;
		double maxPower = 1.0;
	public:
		FeedbackController(/* args */) {}

		virtual double update(double input) { return 0; }

		virtual void reset() {}

		void setTarget(double target) {
			this->target = target;
		}

		double getTarget() {
			return target;
		}

		void setPosition(double target) {
			this->target = target;
		}

		double getPosition() {
			return position;
		}

		double getPower() {
			return power;
		}

		void setMaxPower(double maxPower) {
			this->maxPower = maxPower;
		}

		double getMaxPower() {
			return maxPower;
		}

		~FeedbackController() {}
	};
} // namespace Pronounce
