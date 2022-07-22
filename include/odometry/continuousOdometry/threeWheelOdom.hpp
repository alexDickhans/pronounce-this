#pragma once

#include "continuousOdometry.hpp"
#include "position/odomWheel.hpp"
#include "utils/vector.hpp"
#include "utils/utils.hpp"
#include "utils/pose2d.hpp"

namespace Pronounce
{
    class ThreeWheelOdom : public Odometry {
    private:
        OdomWheel* leftWheel, * rightWheel, * backWheel;
        double leftOffset, rightOffset, backOffset;

		pros::Imu* imu;
		bool useImu = false;

		double maxMovement = 0;
    public:
        ThreeWheelOdom();
        ThreeWheelOdom(OdomWheel* leftWheel, OdomWheel* rightWheel, OdomWheel* backWheel);
        ThreeWheelOdom(OdomWheel* leftWheel, OdomWheel* rightWheel, OdomWheel* backWheel, pros::Imu* imu);

        void update();

        void reset(Pose2D* position) {
            this->setPosition(position);
            this->setResetPosition(position);
            this->leftWheel->reset();
            this->rightWheel->reset();
            this->backWheel->reset();
        }

		double getMaxMovement() {
			return maxMovement;
		}

		void setMaxMovement(double maxMovement) {
			this->maxMovement = maxMovement;
		}

        double getLeftOffset() {
            return leftOffset;
        }

        void setLeftOffset(double leftOffset) {
            this->leftOffset = leftOffset;
        }

        double getRightOffset() {
            return rightOffset;
        }

        void setRightOffset(double rightOffset) {
            this->rightOffset = rightOffset;
        }

        double getBackOffset() {
            return backOffset;
        }

        void setBackOffset(double backOffset) {
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

