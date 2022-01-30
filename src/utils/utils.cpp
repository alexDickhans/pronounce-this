#include "utils.hpp"

namespace Pronounce
{
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
        while (abs(angle1) > M_PI) angle1 += M_PI * 2.0 * -signum_c(angle1);
        while (abs(angle2) > M_PI) angle2 += M_PI * 2.0 * -signum_c(angle2);
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

    // Basically std::format, but only added in C++20
    // https://stackoverflow.com/questions/2342162/stdstring-formatting-like-sprintf

} // namespace Pronounce
