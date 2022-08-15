#pragma once

#include <cmath>
#include <vector>
#include "units/units.hpp"

// TODO: test code
// TODO: Check implemntation
// TODO: add docstrings
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

	class LinearInterpolator {
	private:
		std::vector<double> keys;
		std::vector<double> values;
	public:
		LinearInterpolator();

		void add(double key, double value) {
			for (int i = 0; i < keys.size(); i++) {
				if (keys.at(i) > key) {
					std::vector<double>::iterator index = keys.begin();
					std::advance(index, i);
					keys.insert(index, key);

					std::vector<double>::iterator index = values.begin();
					std::advance(index, i);
					values.insert(index, value);
					return;
				}
			}

			keys.emplace_back(key);
			values.emplace_back(value);
		}

		double get(double key) {
			if (key < keys.at(0)) {
				return lerp(values.at(0), values.at(1), (key - keys.at(0)) /  (keys.at(1) - keys.at(0)));
			} else if (key > keys.at(keys.size() - 1)) {
				return lerp(values.at(keys.size() - 2), values.at(keys.size() - 1), (key - keys.at(keys.size() - 1)) /  (keys.at(keys.size() - 2) - keys.at(keys.size() - 1)));
			}
			
			for (int i = 1; i < keys.size(); i ++) {
				if (keys.at(i-1) < key && keys.at(i) > key) {
					return lerp(keys.at(i-1), keys.at(i), (key - keys.at(keys.size() - 1)) /  (keys.at(keys.size() - 2) - keys.at(keys.size() - 1)));
				}
			}

		} 

		~LinearInterpolator();
	};	
} // namespace Pronounce
