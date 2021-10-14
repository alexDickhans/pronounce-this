#include "position.hpp"

namespace Pronounce {
    Position::Position() {
        this->X = 0;
        this->Y = 0;
        this->theta = 0;
    }

    Position::Position(double theta) {
        this->X = 0;
        this->Y = 0;
        this->theta = theta;
    }

    Position::Position(double X, double Y) {
        this->X = X;
        this->Y = Y;
        this->theta = 0;
    }

    Position::Position(double X, double Y, double theta) {
        this->X = X;
        this->Y = Y;
        this->theta = theta;
    }

    Position::~Position() {
    }
} // namespace Pronounce
