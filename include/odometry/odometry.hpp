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

		double getX() {
			return this->position->getX();
		}

		double getY() {
			return this->position->getY();
		}

		double getTheta() {
			return this->position->getTheta();
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
