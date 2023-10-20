#pragma once

#include <memory>
#include <string>
#include <stdexcept>
#include <cmath>
#include <algorithm>
#include <bits/stdc++.h>
//#include "api.h"

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
	double lerp(double a, double b, double t);

	double toRadians(double degrees);
	double toDegrees(double radians);
	double angleDifference(double angle1, double angle2);

	/**
	 * @brief Returns the sign of a number
	 * 
	 * @param x Number 
	 * @return double Sign of the number -1, 1, or 0
	 */
	double signnum_c(double x);

	/**
	 * @brief A function like processing's map function. Maps a value from one range to another.
	 *
	 * @param value Value to map
	 * @param start1 Start of input range
	 * @param stop1 End of input range
	 * @param start2 Start of output range
	 * @param stop2 End of output range
	 * @return double Mapped value
	 */
	double map(double value, double start1, double stop1, double start2, double stop2);
	
	double findMedian(double arr[], int size);

//	double getDistanceSensorMedian(pros::Distance &distance, int samples);

	unsigned int factorial(const unsigned int& x);

#ifdef SIM
	double clamp(double value, double min, double max) {
		return std::min(std::max(value, min), max);
	}

	double max(double a, double b) {
		return a > b ? a : b;
	}
#else
	using std::max;
	using std::clamp;
#endif // SIM

	template<typename ... Args>
	std::string string_format(const std::string& format, Args ... args) {
		int size_s = std::snprintf(nullptr, 0, format.c_str(), args ...) + 1; // Extra space for '\0'
		if (size_s <= 0) { throw std::runtime_error("Error during formatting."); }
		auto size = static_cast<size_t>(size_s);
		auto buf = std::make_unique<char[]>(size);
		std::snprintf(buf.get(), size, format.c_str(), args ...);
		return std::string(buf.get(), buf.get() + size - 1); // We don't want the '\0' inside
	}
} // namespace Pronounce


