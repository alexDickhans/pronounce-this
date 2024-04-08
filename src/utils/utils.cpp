#include "utils/utils.hpp"

namespace Pronounce
{
	double lerp(double a, double b, double t) {
		return a + t * (b - a);
	}

	double signnum_c(double x) {
        if (x > 0.0) return 1.0;
        if (x < 0.0) return -1.0;
        return x;
    }

    double angleDifference(double angle1, double angle2, size_t maxLoops) {
        double difference = angle1 - angle2;
		size_t loopCount = 0;
        while (fabs(difference) > M_PI) {
			difference += M_PI * 2.0 * -signnum_c(difference);
			loopCount ++;
		}
        return difference;
    }

    double map(double value, double start1, double stop1, double start2, double stop2) {
        double result = start2 + (stop2 - start2) * ((value - start1) / (stop1 - start1));
        if (std::isnan(result)) {
            return 0.0;
        }
        return result;
    }
	
	double findMedian(double arr[], int size) {
		std::sort(arr, arr+size);
		if (size % 2 != 0)
			return (double)arr[size/2];
		return (double)(arr[(size-1)/2] + arr[size/2])/2.0;
	}

	double mean(std::vector<double> array) {
		double sum = 0.0;

		for (const auto &item: array) {
			sum += item;
		}

		return sum/array.size();
	}

	double mean(std::vector<int32_t> array) {
		double sum = 0.0;

		for (const auto &item: array) {
			sum += static_cast<double>(item);
		}

		return sum/array.size();
	}

#ifndef SIM
	double getDistanceSensorMedian(pros::Distance &distance, int samples, double defaultResult) {
		double array[samples];

		array[0] = distance.get();

		bool hasFix = false;

		for (int i = 1; i < samples; i++) {
			pros::Task::delay(20);
			double currentDistance = distance.get();
			if (finite(currentDistance)) {
				hasFix = true;
				array[i] = currentDistance;
			}
		}

		if (hasFix) {
			return findMedian(array, samples);
		} else {
			return defaultResult;
		}
	}
#endif

	unsigned int factorial(const unsigned int& x) {
		if (x < 2)
			return 1;

		unsigned int result = 1;

		for (unsigned int i = x; i >= 0; i--) {
			result *= i;
		}

		return result;
	}

    // Basically std::format, but only added in C++20
    // https://stackoverflow.com/questions/2342162/stdstring-formatting-like-sprintf

} // namespace Pronounce
