#include "threeWheelOdom.hpp"

namespace Pronounce {
    ThreeWheelOdom::ThreeWheelOdom(/* args */) : Odometry() {
    }

    ThreeWheelOdom::ThreeWheelOdom(OdomWheel* leftWheel, OdomWheel* rightWheel, OdomWheel* backWheel) : Odometry() {
        this->leftWheel = leftWheel;
        this->rightWheel = rightWheel;
        this->backWheel = backWheel;
    }

    void ThreeWheelOdom::update() {
        leftWheel->update();
        rightWheel->update();
        backWheel->update();

        double deltaLeft = leftWheel->getChange();
        double deltaRight = rightWheel->getChange();
        double deltaBack = backWheel->getChange();

        Position* lastPosition = this->getPosition();

        double lastAngle = lastPosition->getTheta();
        double angleChange = (deltaLeft - deltaRight) / (leftOffset + rightOffset);
        double currentAngle = lastAngle + angleChange;
        double averageOrientation = lastAngle + (angleChange / 2);

        Vector localOffset;
        if (abs(angleChange) < 0.05) {
            localOffset = Vector(new Point(deltaBack, deltaRight));
        }
        else {
            localOffset = Vector(new Point((deltaBack / angleChange) + backOffset, (deltaRight / angleChange) + rightOffset));
        }

        // Rotate vector
        localOffset.setAngle(localOffset.getAngle() - averageOrientation);

        lastPosition->add(localOffset.getCartesian());
        lastPosition->setTheta(currentAngle);

        this->setPosition(lastPosition);
    }

    ThreeWheelOdom::~ThreeWheelOdom() {
    }
} // namespace Pronounce
