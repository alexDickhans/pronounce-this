#pragma once

#include "utils/utils.hpp"

namespace Pronounce {
	class BangBang {
	private:
		double minimumDifference;
		bool reversable;
		double outputStrength;
		double setPoint = 0;
		double lastInput = 0;
	public:
		BangBang();
		BangBang(double minimumDifference);
		BangBang(double minimumDifference, bool reversable);
		BangBang(double minimumDifference, bool reversable, double outputStrength);

		/**
		 * @brief Updates the controller, input the current value, and it will return the output
		 * 
		 * @param input The current state
		 * @return double The output at that state
		 */
		double update(double input);

		double getSetPoint() {
			return setPoint;
		}

		void setSetPoint(double setPoint) {
			this->setPoint = setPoint;
		}

		double getMinimumDifference() {
			return minimumDifference;
		}

		void setMinimumDifference(double minimumDifference) {
			this->minimumDifference = minimumDifference;
		}

		bool isReversable() {
			return reversable;
		}

		void setReversable(bool reversable) {
			this->reversable = reversable;
		}

		double getOutputStrength() {
			return outputStrength;
		}

		void setOutputStrength(double outputStrength) {
			this->outputStrength = outputStrength;
		}

		bool isInRange() {
			return abs(this->setPoint - this->lastInput) < minimumDifference;
		}

		~BangBang();
	};
} // namespace Pronounce