#pragma once

#include "continuousOdometry.hpp"
#include "position/odomWheel.hpp"
#include "utils/vector.hpp"
#include "utils/utils.hpp"
#include "utils/pose2d.hpp"

// TODO: Add velocity to calculations
// TODO: Add docstrings
// TODO: Add comments
// TODO: Make sure the code is clean

namespace Pronounce {
    class ThreeWheelOdom : public ContinuousOdometry {
    private:
        OdomWheel* leftWheel, * rightWheel, * backWheel;
        QLength leftOffset, rightOffset, backOffset;

		pros::Imu* imu;
		bool useImu = false;

		QLength maxMovement = 0.0;
    public:
        ThreeWheelOdom();
        ThreeWheelOdom(OdomWheel* leftWheel, OdomWheel* rightWheel, OdomWheel* backWheel);
        ThreeWheelOdom(OdomWheel* leftWheel, OdomWheel* rightWheel, OdomWheel* backWheel, pros::Imu* imu);

        void update();

        void reset(Pose2D* position) {
            this->setPose(position);
            this->setResetPose(position);
            this->leftWheel->reset();
            this->rightWheel->reset();
            this->backWheel->reset();
        }

		QLength getMaxMovement() {
			return maxMovement;
		}

		void setMaxMovement(QLength maxMovement) {
			this->maxMovement = maxMovement;
		}

        QLength getLeftOffset() {
            return leftOffset;
        }

        void setLeftOffset(QLength leftOffset) {
            this->leftOffset = leftOffset;
        }

        QLength getRightOffset() {
            return rightOffset;
        }

        void setRightOffset(QLength rightOffset) {
            this->rightOffset = rightOffset;
        }

        QLength getBackOffset() {
            return backOffset;
        }

        void setBackOffset(QLength backOffset) {
            this->backOffset = backOffset;
        }

        OdomWheel* getLeftWheel() {
            return leftWheel;
        }

        void setLeftWheel(OdomWheel* leftWheel) {
            this->leftWheel = leftWheel;
        }

        OdomWheel* getRightWheel() {
            return rightWheel;
        }

        void setRightWheel(OdomWheel* rightWheel) {
            this->rightWheel = rightWheel;
        }

        OdomWheel* getBackWheel() {
            return backWheel;
        }

        void setBackWheel(OdomWheel* backWheel) {
            this->backWheel = backWheel;
        }

        ~ThreeWheelOdom();
    };
} // namespace Pronounce

