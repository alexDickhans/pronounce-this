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

	ThreeWheelOdom::ThreeWheelOdom(OdomWheel* leftWheel, OdomWheel* rightWheel, OdomWheel* backWheel, pros::Imu* imu) : Odometry() {
		this->leftWheel = leftWheel;
		this->rightWheel = rightWheel;
		this->backWheel = backWheel;
		this->imu = imu;
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
		double currentAngle = 0;

		if (useImu && imu != nullptr) {
			currentAngle = toRadians(imu->get_rotation());
		}
		else {
			currentAngle = this->getResetPosition()->getTheta() + (leftWheel->getPosition() - rightWheel->getPosition()) / (leftOffset + rightOffset);
		}

		double deltaAngle = (deltaLeft - deltaRight) / (leftOffset + rightOffset);
		double averageOrientation = lastAngle + (deltaAngle / 2);

		Point localOffset;

		if (deltaAngle != 0.0) {
			double rotationAdjustment = 2 * sin(deltaAngle / 2);
			localOffset = Point(((deltaBack/deltaAngle) + backOffset) * rotationAdjustment, ((deltaRight/deltaAngle) + rightOffset) * rotationAdjustment);
		} else {
			// Calculate the local offset then translate it to the global offset
			localOffset = Point(deltaBack, deltaRight);
		}

		// Rotate local offset
		double rotationCos = cos(averageOrientation);
		double rotationSin = sin(averageOrientation);

		localOffset = Point(localOffset.getX() * rotationCos + localOffset.getY() * rotationSin, - localOffset.getX() * rotationSin + localOffset.getY() * rotationCos);

		// Add localOffset to the global offset
		lastPosition->add(localOffset);
		lastPosition->setTheta(fmod(angleDifference(currentAngle, 0) + M_PI * 2, M_PI * 2));

		if (Vector(&localOffset).getMagnitude() > maxMovement && maxMovement != 0) {
			return;
		}

		// Update the position
		this->setPosition(lastPosition);
	}

	ThreeWheelOdom::~ThreeWheelOdom() {
	}
} // namespace Pronounce
