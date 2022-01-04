#include "threeWheelOdom.hpp"

namespace Pronounce {
    ThreeWheelOdom::ThreeWheelOdom(/* args */) : Odometry() {
		this->reset(new Position());
    }

    ThreeWheelOdom::ThreeWheelOdom(OdomWheel* leftWheel, OdomWheel* rightWheel, OdomWheel* backWheel) : Odometry() {
        this->leftWheel = leftWheel;
        this->rightWheel = rightWheel;
        this->backWheel = backWheel;
		this->reset(new Position());
    }

    void ThreeWheelOdom::update() {
        // Update the wheel positions
        leftWheel->update();
        rightWheel->update();
        backWheel->update();

        // Get the current movement of odometry wheels
        double deltaLeft = leftWheel->getChange();
        double deltaRight = rightWheel->getChange();
        double deltaBack = backWheel->getChange();

        // Get the last robot position
        Position* lastPosition = this->getPosition();

        // Calculate the change in orientation
        double lastAngle = lastPosition->getTheta();
        double currentAngle = this->getResetPosition()->getTheta() + (leftWheel->getPosition() - rightWheel->getPosition()) / (leftOffset + rightOffset);
        double angleChange = angleDifference(currentAngle, lastAngle);
        double averageOrientation = lastAngle + (angleChange / 2);

        // Calculate the local offset then translate it to the global offset
        Vector localOffset = Vector(new Point(deltaBack + (angleChange * backOffset), deltaRight + (angleChange * rightOffset)));

        // Rotate vector
        localOffset.rotate(-averageOrientation);

        // Add localOffset to the global offset
        lastPosition->add(localOffset.getCartesian());
        lastPosition->setTheta(fmod(currentAngle + M_PI * 2, M_PI * 2));

		if (localOffset.getMagnitude() > maxMovement) {
			return;
		}

        // Print last position
		printf("Left Change: %f, Right change: %f, Back change %f\n", deltaLeft, deltaRight, deltaBack);
        printf("Last position: %f, %f, %f\n", lastPosition->getX(), lastPosition->getY(), lastPosition->getTheta());

        // Update the position
        this->setPosition(lastPosition);
    }

    ThreeWheelOdom::~ThreeWheelOdom() {
    }
} // namespace Pronounce
