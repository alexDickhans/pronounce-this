#pragma once

#include "utils/position.hpp"

namespace Pronounce {
    class Odometry {
    private:
        Position* position;
        Position* resetPosition;
    public:
        Odometry();
        Odometry(Position* position);

        Position* getPosition() {
            return this->position;
        }

        void setPosition(Position* position) {
            this->position->operator=(position);
        }

        Position* getResetPosition() {
            return this->resetPosition;
        }

        void setResetPosition(Position* resetPosition) {
            this->resetPosition->operator=(resetPosition);
        }

        virtual void update() {};

        virtual void reset(Position* position) {
            this->position->operator=(position);
            this->resetPosition->operator=(position);
        }
        
        void reset() {
            this->reset(new Position());
        }

        ~Odometry();
    };    
} // namespace Pronounce
