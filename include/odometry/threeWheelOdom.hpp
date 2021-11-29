#pragma once

#include "odometry.hpp"
#include "position/odomWheel.hpp"
#include "utils/vector.hpp"

namespace Pronounce
{
    class ThreeWheelOdom : public Odometry {
    private:
        OdomWheel* leftWheel, * rightWheel, * backWheel;
        double leftOffset, rightOffset, backOffset;
    public:
        ThreeWheelOdom();
        ThreeWheelOdom(OdomWheel* leftWheel, OdomWheel* rightWheel, OdomWheel* backWheel);

        void update();

        void reset(Position* position) {
            Odometry::reset();
            this->leftWheel->reset();
            this->rightWheel->reset();
            this->backWheel->reset();
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

