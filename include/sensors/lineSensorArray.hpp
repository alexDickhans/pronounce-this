#pragma once

#include "api.h"
#include <vector>

namespace Pronounce {
	class LineSensorArray
	{
	private:
		std::vector<pros::ADILineSensor&> lineSensors;
	public:
		LineSensorArray() {}

		double getAverage() {
			int32_t total = 0;

			for (int i = 0; i < lineSensors.size(); i++) {
				total += lineSensors.at(i).get_value();
			}

			return total / lineSensors.size();
		}

		double getMax() {
			int32_t max = 0;

			for (int i = 0; i < lineSensors.size(); i++) {
				int32_t value = lineSensors.at(i).get_value();
				if (value > max) {
					max = value;
				}
			}
			
			return max;
		}

		double getMin() {
			int32_t min = lineSensors.at(1).get_value();

			// We can skip the first value because we are already checking that
			for (int i = 1; i < lineSensors.size(); i ++) {
				int32_t value = lineSensors.at(i).get_value();
				if (value < min) {
					min = value;
				}
			}

			return min;
		}

		void addLineSensor(pros::ADILineSensor& lineSensor) {
			this->lineSensors.emplace_back(lineSensor);
		}

		pros::ADILineSensor& getLineSensor(uint32_t index) {
			return lineSensors.at(index);
		}

		~LineSensorArray();
	};
} // namespace Pronoucne
