#include "threeWheelOdom.hpp"

namespace Pronounce {
    ThreeWheelOdom::ThreeWheelOdom(/* args */) {
    }

    ThreeWheelOdom::ThreeWheelOdom(OdomWheel* leftWheel, OdomWheel* rightWheel, OdomWheel* backWheel) {
        this->leftWheel = leftWheel;
        this->rightWheel = rightWheel;
        this->backWheel = backWheel;
    }

    void ThreeWheelOdom::update() {
        double deltaLeft = leftWheel->getChange();
        double deltaRight = rightWheel->getChange();
        double deltaBack = backWheel->getChange();

        Position* lastPosition = this->getPosition();
        Position newPosition = *this->getPosition();

        double lastAngle = lastPosition->getTheta();
        double angleChange = (deltaLeft - deltaRight) / (leftOffset / rightOffset);
        double currentAngle = lastAngle + angleChange;
        double averageOrientation = lastAngle + (angleChange / 2);

        Vector localOffset;
        if (angleChange == 0) {
            localOffset = Vector(deltaBack, deltaRight);
        }
        else {
            localOffset = Vector((deltaBack / angleChange) + backOffset, (deltaRight / angleChange) + rightOffset);
        }

        // Rotate vector
        localOffset.setAngle(localOffset.getAngle() - averageOrientation);

        newPosition += localOffset.getCartesian();
        newPosition.setTheta(currentAngle);

        this->setPosition(&newPosition);
    }

    ThreeWheelOdom::~ThreeWheelOdom() {
    }
} // namespace Pronounce
