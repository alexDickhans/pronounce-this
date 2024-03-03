#pragma once

#include <utility>

#include "lineSensor.hpp"
#include "pros/adi.hpp"

namespace Loco {
	class ADILineSensor : public LineSensor {
	private:
		pros::ADILineSensor& lineSensor;
		uint32_t lineMin;
	public:
		ADILineSensor(pros::ADILineSensor& lineSensor, uint32_t lineMin, FieldModel& fieldModel, Eigen::Vector2d sensorPosition) : lineSensor(lineSensor),
		                                                                                                         LineSensor(fieldModel, std::move(sensorPosition)) {
			this->lineMin = lineMin;
		}

		bool getReading() override {
			return lineSensor.get_value() > this->lineMin;
		}
	};
}