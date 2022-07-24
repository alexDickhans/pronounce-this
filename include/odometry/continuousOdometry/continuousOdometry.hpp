#pragma once

#include "utils/pose2d.hpp"
#include "utils/vector.hpp"

namespace Pronounce {
    class ContinuousOdometry {
    private:
        Pose2D* position;
        Pose2D* resetPosition;

		Vector currentVelocity;
    public:
        ContinuousOdometry();
        ContinuousOdometry(Pose2D* position);

        Pose2D* getPosition() {
            return this->position;
        }

		double getX() {
			return this->position->getX();
		}

		double getY() {
			return this->position->getY();
		}

		double getTheta() {
			return this->position->getAngle();
		}

        void setPosition(Pose2D* position) {
            this->position->operator=(position);
        }

        Pose2D* getResetPosition() {
            return this->resetPosition;
        }

        void setResetPosition(Pose2D* resetPosition) {
            this->resetPosition->operator=(resetPosition);
        }
		
		Vector getCurrentVelocity() {
			return this->currentVelocity;
		}

		void setCurrentVelocity(Vector velocity) {
			this->currentVelocity = velocity;
		}

        virtual void update() {};

        virtual void reset(Pose2D* position) {
            this->position->operator=(position);
            this->resetPosition->operator=(position);
			this->currentVelocity.operator=(Vector());
        }
        
        void reset() {
            this->reset(new Pose2D());
        }

        ~ContinuousOdometry();
    };    
} // namespace Pronounce
