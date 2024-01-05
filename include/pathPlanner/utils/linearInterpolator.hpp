#pragma once

#include <cmath>
#include <vector>
#include "../../units/units.hpp"

// TODO: test code
// TODO: Check implemntation
// TODO: add comments

namespace PathPlanner {

	struct KeyValuePair_ {
		double key;
		double value;
	} typedef KeyValuePair;

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
		std::vector<KeyValuePair> values{};
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
			KeyValuePair pair = {key, value};
			values.emplace_back(pair);
		}

		/**
		 * @brief Get the value at the key
		 *
		 * @param key The key to check at
		 * @return double The value at that key
		 */
		double get(double key) {

			if (values.empty()) {
				return 0.0;
			}

			if (values.size() < 2) {
				return values.at(0).value;
			}

			for (int i = 1; i < values.size(); i++) {
				if (key < values.at(0).key) {
					return values.at(i-1).value + (((values.at(i).value - values.at(i-1).value)/(values.at(i).key - values.at(i-1).key)) * (key - values.at(i-1).value));
				}
			}

			return values.at(values.size()-2).value + ((values.at(values.size()-1).value - values.at(values.size()-2).value)/(values.at(values.size()-1).key - values.at(values.size()-2).key)) * (key - values.at(values.size()-2).key);
		}

		void clear() {
			values.clear();
		}

		~LinearInterpolator() = default;
	};
} // namespace Pronounce
