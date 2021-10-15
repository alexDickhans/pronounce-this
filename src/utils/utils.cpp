#include "utils.hpp"

namespace Pronounce
{
    double toRadians(double degrees) {
        return degrees * M_PI / 180;
    }

    double toDegrees(double radians) {
        return radians * 180 / M_PI;
    }

    double mapFunc(double value, double start1, double stop1, double start2, double stop2) {
        return ((value - start1) / (stop1 - start1)) * ((stop2 - start2) + start2);
    }

    double signum_c(double x) {
        if (x > 0.0) return 1.0;
        if (x < 0.0) return -1.0;
        return x;
    }

    // Basically std::format, but only added in C++20
    // https://stackoverflow.com/questions/2342162/stdstring-formatting-like-sprintf

} // namespace Pronounce
