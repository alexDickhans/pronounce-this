#pragma once

#include <cmath>
#include <vector>
#include "../../units/units.hpp"
#include "../../utils/utils.hpp"

// TODO: test code
// TODO: Check implemntation
// TODO: add comments

namespace PathPlanner {

	/**
	 * @brief Linear interpolator class for use in flywheels and other functions
	 *
	 * @authors Alex Dickhans
	 */
	class LinearInterpolator {
	private:
		/**
		 * @brief Pair of keys and values
		 *
		 */
		std::vector<std::pair<double, double>> values;
	public:
		/**
		 * @brief Construct a new Linear Interpolator object
		 *
		 */
		LinearInterpolator() {}

		/**
		 * @brief Add a key and value to the linear interpolator
		 *
		 * @param key The new key to add
		 * @param value The new value to add
		 */
		void add(double key, double value) {
			values.emplace_back(std::pair<double, double>(key, value));
		}

		/**
		 * @brief Get the value at the key
		 *
		 * @param key The key to check at
		 * @return double The value at that key
		 */
		double get(double key) {
			if (key < values.at(0).first) {
				double t = Pronounce::map(key, values.at(0).first, values.at(1).first, 0, 1);
				return Pronounce::lerp(values.at(0).second, values.at(1).second, t);
			}

			for (int i = 0; i < values.size() - 1; i++) {
				if (key > values.at(i).first) {
					double t = Pronounce::map(key, values.at(i).first, values.at(i+1).first, 0, 1);
					return Pronounce::lerp(values.at(i).second, values.at(i+1).second, t);
				}
			}

			double t = Pronounce::map(key, values.at(values.size()-2).first, values.at(values.size()-1).first, 0, 1);
			return Pronounce::lerp(values.at(values.size()-2).second, values.at(values.size()-1).second, t);
		}

		void clear() {
			values.clear();
		}

		~LinearInterpolator() {}
	};
} // namespace Pronounce
