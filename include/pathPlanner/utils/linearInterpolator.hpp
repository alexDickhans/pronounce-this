#pragma once

#include <cmath>
#include <vector>
#include "../../units/units.hpp"

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
		LinearInterpolator() = default;

		/**
		 * @brief Add a key and value to the linear interpolator
		 *
		 * @param key The new key to add
		 * @param value The new value to add
		 */
		void add(double key, double value) {
			values.emplace_back(key, value);
		}

		/**
		 * @brief Get the value at the key
		 *
		 * @param key The key to check at
		 * @return double The value at that key
		 */
		[[nodiscard]] double get(double key) const {
			if (key <= values.at(0).first) {
				return values.at(0).second + ((values.at(1).second - values.at(0).second)/(values.at(1).first - values.at(0).first)) * (key - values.at(1).first);
			}

			for (int i = 0; i < values.size(); i++) {
				if (key <= values.at(i).first || i == values.size()-1) {
					return values.at(i-1).second + ((values.at(i).second - values.at(i-1).second)/(values.at(i).first - values.at(i-1).first)) * (key - values.at(i).first);
				}
			}
		}

		[[nodiscard]] double getIntegral(double key) const { // TODO: implement
			if (key <= values.at(1).first || values.size() < 2) {
				double x = (key - values.at(1).first);
				return 0.5 * values.at(1).second + ((values.at(1).second - values.at(0).second)/(values.at(1).first - values.at(0).first)) * x * x;
			}

			double total = 0.5 * (values.at(1).first - values.at(0).first) * (values.at(1).second - values.at(0).second);

			for (int i = 2; i < values.size(); i++) {
				if (key <= values.at(i).first || i == values.size()-1) {
					double x = (key - values.at(i).first);
					return total + 0.5 * values.at(i).second + ((values.at(i).second - values.at(i-1).second)/(values.at(i).first - values.at(i-1).first)) * x * x;
				}

				total += 0.5 * (values.at(i).first - values.at(i-1).first) * (values.at(1).second - values.at(0).second);
			}
		}

		void clear() {
			values.clear();
		}

		~LinearInterpolator() = default;
	};
} // namespace Pronounce
