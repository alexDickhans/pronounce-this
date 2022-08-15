#pragma once

#include "api.h"
#include <vector>

// TODO: Add positions
// TODO: clean up
// TODO: Add docstrings
// TODO: check code

namespace Pronounce {
	class LineSensorArray {
	private:
		std::vector<std::pair<pros::ADILineSensor&, Point>> lineSensors;
	public:
		LineSensorArray() {}

		double getAverage() {
			int32_t total = 0;

			for (int i = 0; i < lineSensors.size(); i++) {
				total += lineSensors.at(i).first.get_value();
			}

			return total / lineSensors.size();
		}

		std::pair<Point, int32_t> getMax() {
			int32_t max = 0;
			Point position;

			for (int i = 0; i < lineSensors.size(); i++) {
				int32_t value = lineSensors.at(i).first.get_value();
				if (value > max) {
					max = value;
					position = lineSensors.at(i).second;
				}
			}
			
			return std::pair<Point, int32_t>(position, max);
		}

		std::pair<Point, int32_t> getMin() {
			int32_t min = lineSensors.at(1).first.get_value();
			Point position = lineSensors.at(1).second;

			// We can skip the first value because we are already checking that
			for (int i = 1; i < lineSensors.size(); i ++) {
				int32_t value = lineSensors.at(i).first.get_value();
				if (value < min) {
					min = value;
				}
			}

			return std::pair<Point, int32_t>(position, min);
		}

		void addLineSensor(pros::ADILineSensor& lineSensor, Point point) {
			this->lineSensors.emplace_back(std::pair<pros::ADILineSensor&, Point>(lineSensor, point));
		}

		std::pair<pros::ADILineSensor&, Point> getLineSensor(uint32_t index) {
			return lineSensors.at(index);
		}

		~LineSensorArray();
	};
} // namespace Pronoucne
