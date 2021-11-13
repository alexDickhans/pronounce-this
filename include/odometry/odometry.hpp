#pragma once

#include "utils/position.hpp"

namespace Pronounce {
    class Odometry {
    private:
        Position* position;
    public:
        Odometry();
        Odometry(Position* position);

        Position* getPosition() {
            return this->position;
        }

        void setPosition(Position* position) {
            this->position = position;
        }

        virtual void update() {};

        ~Odometry();
    };    
} // namespace Pronounce
