#include "threeWheelOdom.hpp"

namespace Pronounce {
	ThreeWheelOdom::ThreeWheelOdom(/* args */) : ContinuousOdometry() {
		this->reset(new Pose2D());
	}

	ThreeWheelOdom::ThreeWheelOdom(OdomWheel* leftWheel, OdomWheel* rightWheel, OdomWheel* backWheel) : ContinuousOdometry() {
		this->leftWheel = leftWheel;
		this->rightWheel = rightWheel;
		this->backWheel = backWheel;
		this->reset(new Pose2D());
	}

	ThreeWheelOdom::ThreeWheelOdom(OdomWheel* leftWheel, OdomWheel* rightWheel, OdomWheel* backWheel, pros::Imu* imu) : ContinuousOdometry() {
		this->leftWheel = leftWheel;
		this->rightWheel = rightWheel;
		this->backWheel = backWheel;
		this->imu = imu;
		this->reset(new Pose2D());
	}

	void ThreeWheelOdom::update() {
		// Update the wheel positions
		leftWheel->update();
		rightWheel->update();
		backWheel->update();

		// Get the current movement of odometry wheels
		QLength deltaLeft = leftWheel->getChange();
		QLength deltaRight = rightWheel->getChange();
		QLength deltaBack = backWheel->getChange();

		// Get the last robot position
		Pose2D* lastPose = this->getPose();

		// Calculate the change in orientation
		Angle lastAngle = lastPose->getAngle();
		Angle currentAngle = 0.0;

		if (useImu && imu != nullptr) {
			currentAngle = toRadians(imu->get_rotation());
		}
		else {
			currentAngle = this->getResetPose()->getAngle().getValue() + (leftWheel->getPosition() - rightWheel->getPosition()).getValue() / (leftOffset + rightOffset).getValue();
		}

		Angle deltaAngle = (deltaLeft - deltaRight).getValue() / (leftOffset + rightOffset).getValue();
		Angle averageOrientation = lastAngle + (deltaAngle / 2);

		Point localOffset;

		if (deltaAngle.Convert(radian) != 0.0) {
			double rotationAdjustment = 2 * sin(deltaAngle / 2);
			localOffset = Point(((deltaBack/deltaAngle).getValue() + backOffset.getValue()) * rotationAdjustment, ((deltaRight/deltaAngle).getValue() + rightOffset.getValue()) * rotationAdjustment);
		} else {
			// Calculate the local offset then translate it to the global offset
			localOffset = Point(deltaBack, deltaRight);
		}

		// Rotate local offset
		double rotationCos = cos(averageOrientation);
		double rotationSin = sin(averageOrientation);

		localOffset = Point(localOffset.getX().Convert(metre) * rotationCos + localOffset.getY().Convert(metre) * rotationSin, - localOffset.getX().Convert(metre) * rotationSin + localOffset.getY().Convert(metre) * rotationCos);

		// Add localOffset to the global offset
		lastPose->add(localOffset);
		lastPose->setAngle(fmod(angleDifference(currentAngle.Convert(radian), 0) + M_PI * 2, M_PI * 2));

		if (Vector(&localOffset).getMagnitude() > maxMovement && maxMovement.Convert(metre) != 0.0) {
			return;
		}

		// Update the position
		this->setPose(lastPose);
	}

	ThreeWheelOdom::~ThreeWheelOdom() {
	}
} // namespace Pronounce
