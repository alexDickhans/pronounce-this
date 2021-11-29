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
        double currentAngle = this->getResetPosition()->getTheta() + (leftWheel->getPosition() - rightWheel->getPosition()) / (rightOffset + rightOffset);
        double angleChange = currentAngle - lastAngle;
        double averageOrientation = lastAngle + (angleChange / 2);

        Vector localOffset;
        double PI_2 = 2 * M_PI;
        localOffset = Vector(new Point(deltaBack + ((angleChange / PI_2) * PI_2 * backOffset), deltaRight + ((angleChange / PI_2) * PI_2 * rightOffset)));

        // Rotate vector
        localOffset.setAngle(localOffset.getAngle() - averageOrientation);

        lastPosition->add(localOffset.getCartesian());
        lastPosition->setTheta(fmod(currentAngle + M_PI * 2, M_PI * 2));

        this->setPosition(lastPosition);
    }

    ThreeWheelOdom::~ThreeWheelOdom() {
    }
} // namespace Pronounce
