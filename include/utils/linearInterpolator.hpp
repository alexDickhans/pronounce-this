#pragma once

#include <cmath>
#include <vector>
#include "units/units.hpp"

// TODO: test code
// TODO: Check implemntation
// TODO: add comments

namespace Pronounce {
	/**
	 * @brief Simple linear interpolation
	 * 
	 * @param a The start value
	 * @param b The end value
	 * @param t 
	 * @return double The interpolated value
	 */
	double lerp(double a, double b, double t) {
		return a + t * (b - a);
	}

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
			for (int i = 0; i < values.size(); i++) {
				if (values.at(i).second > key) {
					std::vector<std::pair<double, double>>::iterator index = values.begin();
					std::advance(index, i);
					values.insert(index, std::pair<double, double>(key, value));
					return;
				}
			}

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
				return lerp(values.at(0).second, values.at(1).second, (key - values.at(0).first) /  (values.at(1).first - values.at(0).first));
			} else if (key > values.at(values.size() - 1).first) {
				return lerp(values.at(values.size() - 2).second, values.at(values.size() - 1).second, ((key - values.at(values.size() - 1).first) /  (values.at(values.size() - 2).first - values.at(values.size() - 1).second)));
			}
			
			for (int i = 1; i < values.size(); i ++) {
				if (values.at(i-1).first < key && values.at(i).first > key) {
					return lerp(values.at(i-1).first, values.at(i).first, (key - values.at(values.size() - 1).first) /  (values.at(values.size() - 2).first - values.at(values.size() - 1).first));
				}
			}

			return 0.0;
		} 

		~LinearInterpolator() {}
	};	
} // namespace Pronounce
