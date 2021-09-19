#include "utils.hpp"

namespace Pronounce
{
    double toRadians(double degrees) {
        return degrees * M_PI / 180;
    }

    double mapFunc(double value, double start1, double stop1, double start2, double stop2) {
        return ((value - start1) / (stop1 - start1)) * ((stop2 - start2) + start2);
    }

    // Basically std::format, but only added in C++20
    // https://stackoverflow.com/questions/2342162/stdstring-formatting-like-sprintf

} // namespace Pronounce
