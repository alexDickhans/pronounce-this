#pragma once

#include "utils/pose2d.hpp"
#include "utils/vector.hpp"

// TODO: Add docstrings
// TODO: Add comments
// TODO: Clean code

namespace Pronounce {
    class ContinuousOdometry {
    private:
        Pose2D* pose;
        Pose2D* resetPose;

		Vector currentVelocity;
    public:
        ContinuousOdometry();
        ContinuousOdometry(Pose2D* pose);

        Pose2D* getPosition() {
            return this->pose;
        }

		QLength getX() {
			return this->pose->getX();
		}

		QLength getY() {
			return this->pose->getY();
		}

		Angle getAngle() {
			return this->pose->getAngle();
		}

		Pose2D* getPose() {
			return pose;
		}

        void setPose(Pose2D* pose) {
            this->pose->operator=(pose);
        }

        Pose2D* getResetPose() {
            return this->resetPose;
        }

        void setResetPose(Pose2D* resetPose) {
            this->resetPose->operator=(resetPose);
        }
		
		Vector getCurrentVelocity() {
			return this->currentVelocity;
		}

		void setCurrentVelocity(Vector velocity) {
			this->currentVelocity = velocity;
		}

        virtual void update() {};

        virtual void reset(Pose2D* pose) {
            this->pose->operator=(pose);
            this->resetPose->operator=(pose);
			this->currentVelocity.operator=(Vector());
        }
        
        void reset() {
            this->reset(new Pose2D());
        }

        ~ContinuousOdometry();
    };    
} // namespace Pronounce
