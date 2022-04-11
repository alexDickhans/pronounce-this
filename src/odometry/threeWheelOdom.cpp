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

		double deltaAngle = angleDifference(currentAngle, lastAngle);
		double averageOrientation = lastAngle + (deltaAngle / 2);

		Vector localOffset;

		if (abs(deltaAngle) < 0.000001) {
			// Calculate the local offset then translate it to the global offset
			localOffset = Vector(new Point(deltaBack, deltaRight));
		} else {
			double rotationAdjustment = 2 * sin(deltaAngle / 2);
			localOffset = Vector(new Point((deltaBack/deltaAngle) + backOffset, (deltaRight/deltaAngle) + rightOffset));
			localOffset.scale(rotationAdjustment);
		}

		// Rotate vector
		localOffset.rotate(-averageOrientation);

		// Add localOffset to the global offset
		lastPosition->add(localOffset.getCartesian());
		lastPosition->setTheta(fmod(currentAngle + M_PI * 2, M_PI * 2));

		if (localOffset.getMagnitude() > maxMovement && maxMovement != 0) {
			return;
		}

		// Update the position
		this->setPosition(lastPosition);
	}

	ThreeWheelOdom::~ThreeWheelOdom() {
	}
} // namespace Pronounce
