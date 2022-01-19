#include "bangBang.hpp"

namespace Pronounce {
	BangBang::BangBang() : BangBang(0, false, 0) {
	}

	BangBang::BangBang(double minimumDifference) : BangBang(minimumDifference, false, 0) {
	}

	BangBang::BangBang(double minimumDifference, bool reversable) : BangBang(minimumDifference, reversable, 0){
	}

	BangBang::BangBang(double minimumDifference, bool reversable, double outputStrength) {
		this->minimumDifference = minimumDifference;
		this->reversable = reversable;
		this->outputStrength = outputStrength;
	}

	double BangBang::update(double input) {
		// Get the difference between the set point and the current value
		double difference = setPoint - input;

		// Set the lastInput variable to this loop's input.
		this->lastInput = input;
		
		// Change behavior if the controller is not reversable
		if (reversable) {
			if (abs(difference) > minimumDifference) {
				// Return the opisite of the way that we are so we go that way
				return outputStrength * -signnum_c(difference);
			} else {
				// Don't return anything if the value is null.
				return 0;
			}
		} else {
			if (difference < -minimumDifference) {
				// Return the opisite of the way that we are so we go that way	
				return outputStrength * -1;
			} else {
				return 0;
			}
		}
	}

	BangBang::~BangBang() {
	}
} // namespace Pronounce
