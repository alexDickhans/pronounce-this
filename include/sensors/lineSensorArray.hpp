#pragma once

#include "api.h"
#include <vector>

// TODO: Add docstrings

namespace Pronounce {
	/**
	 * @brief An array of line sensors with positions around the robot
	 * 
	 */
	class LineSensorArray {
	private:
		/**
		 * @brief The array of pairs of line sensors and points on the robot
		 * 
		 */
		std::vector<std::pair<pros::ADILineSensor&, Point>> lineSensors;
	public:
		/**
		 * @brief Construct a new Line Sensor Array object
		 * 
		 */
		LineSensorArray() {}

		/**
		 * @brief Get the Average of all the line sensors
		 * 
		 * @return double The average of all the sensors
		 */
		double getAverage() {
			int32_t total = 0;

			for (int i = 0; i < lineSensors.size(); i++) {
				total += lineSensors.at(i).first.get_value();
			}

			return total / lineSensors.size();
		}

		/**
		 * @brief Get the Max line sensor
		 * 
		 * @return std::pair<Point, int32_t> Point - the point of the line sensor, int32_t the highest value
		 */
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

		/**
		 * @brief Get the Min line sensor
		 * 
		 * @return std::pair<Point, int32_t> Point - the point of the line sensor, int32_t the minimum value
		 */
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

		/**
		 * @brief Add a line sensor to the array
		 * 
		 * @param lineSensor The line sensor reference 
		 * @param point The position of that line sensor on the robot
		 */
		void addLineSensor(pros::ADILineSensor& lineSensor, Point point) {
			this->lineSensors.emplace_back(std::pair<pros::ADILineSensor&, Point>(lineSensor, point));
		}

		/**
		 * @brief Get the Line Sensor object at a index
		 * 
		 * @param index The index in the array at which to get a line sensor from
		 * @return std::pair<pros::ADILineSensor&, Point> 
		 */
		std::pair<pros::ADILineSensor&, Point> getLineSensor(uint32_t index) {
			return lineSensors.at(index);
		}

		~LineSensorArray();
	};
} // namespace Pronoucne
