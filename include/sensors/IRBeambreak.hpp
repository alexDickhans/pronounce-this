#pragma once

#include "api.h"

// TODO: Test code
// TODO: add docstrings
// TODO: Comments

namespace Pronounce {
	class IRBeambreak {
	private:
		pros::ADIAnalogIn* irSensor;
		int threshold;
	public:
		IRBeambreak(pros::ADIAnalogIn* irSensor) {
			this->irSensor = irSensor;
		}

		bool getValue() {
			return irSensor->get_value() < threshold;
		}

		pros::ADIAnalogIn* getIrSensor() {
			return irSensor;
		}

		void setIrSensor(pros::ADIAnalogIn* irSensor) {
			this->irSensor = irSensor;
		}
		
		~IRBeambreak() {}
	};	
} // namespace Pronounce
