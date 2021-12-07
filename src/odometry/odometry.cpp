#include "odometry.hpp"

namespace Pronounce {
    Odometry::Odometry() {
        this->position = new Position();
        this->resetPosition = new Position();
    }

    Odometry::Odometry(Position* position) {
        this->position = position;
        this->resetPosition = new Position();
    }
    
    Odometry::~Odometry() {
    }
} // namespace Pronounce

