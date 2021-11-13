#include "odometry.hpp"

namespace Pronounce {
    Odometry::Odometry() {
        this->position = new Position();
    }

    Odometry::Odometry(Position* position) {
        this->position = position;
    }
    
    Odometry::~Odometry() {
    }
} // namespace Pronounce

