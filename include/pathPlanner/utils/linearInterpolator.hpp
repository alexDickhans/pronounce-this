#pragma once

#include <cmath>
#include <vector>
#include "../../units/units.hpp"
#include <string>
#include <iostream>

// TODO: test code
// TODO: Check implemntation
// TODO: add comments

namespace PathPlanner {
	template<typename ... Args>
	std::string string_format(const std::string& format, Args ... args) {
		int size_s = std::snprintf(nullptr, 0, format.c_str(), args ...) + 1; // Extra space for '\0'
		if (size_s <= 0) { throw std::runtime_error("Error during formatting."); }
		auto size = static_cast<size_t>(size_s);
		auto buf = std::make_unique<char[]>(size);
		std::snprintf(buf.get(), size, format.c_str(), args ...);
		return std::string(buf.get(), buf.get() + size - 1); // We don't want the '\0' inside
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

			if (values.empty()) {
				return 0.0;
			} else if (values.size() == 1) {
				return values.at(0).second;
			}

			if (key <= values.at(1).first || values.size() < 2) {
				return values.at(1).second + ((values.at(1).second - values.at(0).second)/(values.at(1).first - values.at(0).first)) * (key - values.at(1).first);
			}

			for (int i = 2; i < values.size(); i++) {
				if (key <= values.at(i).first || i == values.size()-1) {
					return values.at(i).second + ((values.at(i).second - values.at(i-1).second)/(values.at(i).first - values.at(i-1).first)) * (key - values.at(i).first);
				}
			}

			return 0;
		}

		[[nodiscard]] double getIntegral(double key) const {

			if (values.empty()) {
				return 0.0;
			} else if (values.size() == 1) {
				return values.at(0).second * (key-values.at(0).first);
			}

			if (key <= values.at(1).first || values.size() < 2) {
				double x = std::min((key - values.at(0).first), values.at(1).first - values.at(0).first);
				return x * values.at(0).second + 0.5 * ((values.at(1).second - values.at(0).second)/(values.at(1).first - values.at(0).first)) * x * x;
			}

			double total = 0.5 * (values.at(1).first - values.at(0).first) * (values.at(1).second - values.at(0).second);

			for (int i = 2; i < values.size(); i++) {
				if (key <= values.at(i).first || i == values.size()-1) {
					double x = (key - values.at(i-1).first);
					return total + x * values.at(i-1).second + 0.5 * ((values.at(i).second - values.at(i-1).second)/(values.at(i).first - values.at(i-1).first)) * x * x;
				}

				total = total + 0.5 * (values.at(i).first - values.at(i-1).first) * (values.at(i).second + values.at(i-1).second);
			}

			return total;
		}

		void clear() {
			values.clear();
		}

		[[nodiscard]] std::string to_string() const {
			std::string buf;

			for (const auto &valuePair: values) {
				buf += string_format("%f: %f\n", valuePair.first, valuePair.second);
			}

			return buf;
		}

		~LinearInterpolator() = default;
	};
} // namespace Pronounce
