#include "utils/utils.hpp"

namespace Pronounce
{
	double lerp(double a, double b, double t) {
		return a + t * (b - a);
	}
	
    double toRadians(double degrees) {
        return degrees * M_PI / 180;
    }

    double toDegrees(double radians) {
        return radians * 180 / M_PI;
    }

    double signum_c(double x) {
        if (x > 0.0) return 1.0;
        if (x < 0.0) return -1.0;
        return x;
    }

	double signnum_c(double x) {
        if (x > 0.0) return 1.0;
        if (x < 0.0) return -1.0;
        return x;
    }

    double angleDifference(double angle1, double angle2) {
        double difference = angle1 - angle2;
        while (abs(difference) > M_PI) difference += M_PI * 2.0 * -signum_c(difference);
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

	double getDistanceSensorMedian(pros::Distance &distance, int samples) {
		double array[samples];

		array[0] = distance.get();

		for (int i = 1; i < samples; i++) {
			pros::Task::delay(20);
			array[i] = distance.get();
		}

		return findMedian(array, samples);
	}

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
