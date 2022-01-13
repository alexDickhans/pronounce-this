#include "pointUtil.hpp"

namespace Pronounce {
    Point::Point() {
        this->x = 0.0;
        this->y = 0.0;
    }

    Point::Point(double x, double y) {
        this->x = x;
        this->y = y;
    }

    Point::~Point() {
    }
} // namespace Pronounce
