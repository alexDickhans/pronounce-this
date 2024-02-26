#pragma once

#include "timeOfFlight.hpp"
#include "pros/distance.hpp"

#define LIDAR_MAX_DISTANCE 1.3_m

namespace Loco {

	class DistanceSensor : public TimeOfFlight {
		pros::Distance& distance;
	public:
		DistanceSensor() = delete;
		explicit DistanceSensor(pros::Distance& distance, FieldModel& model, QLength maxLength = LIDAR_MAX_DISTANCE) : distance(distance),
																																TimeOfFlight(model, maxLength) {

		}

        void configure() {}

		QLength getReading() override {
			auto value = distance.get();

			if (value == 9999) {
				return std::numeric_limits<double>::quiet_NaN();
			}

			return value * 1_mm;
		}

		QLength getStd() override {
			return distance.get_confidence() * 1_mm;
		}
	};
}
