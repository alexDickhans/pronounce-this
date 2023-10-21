#include "utils/point.hpp"

namespace Pronounce {
    Point::Point() {
        this->x = 0.0;
        this->y = 0.0;
    }

    Point::Point(QLength x, QLength y) {
        this->x = x;
        this->y = y;
    }

    Point::~Point() {
    }
} // namespace Pronounce
