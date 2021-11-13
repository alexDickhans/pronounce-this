#include "position.hpp"

namespace Pronounce {
    Position::Position() : Point() {
        this->theta = 0;
    }

    Position::Position(double theta) : Point() {
        this->theta = theta;
    }

    Position::Position(double x, double y) : Point(x, y) {
        this->theta = 0;
    }

    Position::Position(double x, double y, double theta) : Point(x, y) {
        this->theta = theta;
    }

    Position::~Position() {
    }
} // namespace Pronounce
