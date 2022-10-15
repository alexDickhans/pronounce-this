#include "threeWheelOdom.hpp"

namespace Pronounce {
	ThreeWheelOdom::ThreeWheelOdom(/* args */) : ContinuousOdometry() {
		this->reset(Pose2D());
	}

	ThreeWheelOdom::ThreeWheelOdom(std::shared_ptr<OdomWheel> leftWheel, std::shared_ptr<OdomWheel> rightWheel, std::shared_ptr<OdomWheel> backWheel) : ContinuousOdometry() {
		this->leftWheel = leftWheel;
		this->rightWheel = rightWheel;
		this->backWheel = backWheel;
		this->reset(Pose2D());
	}

	ThreeWheelOdom::ThreeWheelOdom(std::shared_ptr<OdomWheel> leftWheel, std::shared_ptr<OdomWheel> rightWheel, std::shared_ptr<OdomWheel> backWheel, std::shared_ptr<Orientation> externalOrientation) : ContinuousOdometry() {
		this->leftWheel = leftWheel;
		this->rightWheel = rightWheel;
		this->backWheel = backWheel;
		this->externalOrientation = externalOrientation;
		this->reset(Pose2D());
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
		Pose2D lastPose = this->getPose();

		// Calculate the change in orientation
		Angle lastAngle = lastPose.getAngle();
		Angle currentAngle = 0.0;

		// If we are using external orientation and it is set we will use that values instead of the current angle
		if (useExternalOrientation && externalOrientation != nullptr) {
			currentAngle = externalOrientation->getAngle();
		}
		else {
			currentAngle = this->getResetPose().getAngle().getValue() + (leftWheel->getPosition() - rightWheel->getPosition()).getValue() / (leftOffset + rightOffset).getValue();
		}

		// Calculate some values to use later
		Angle deltaAngle = (deltaLeft - deltaRight).getValue() / (leftOffset + rightOffset).getValue();
		Angle averageOrientation = lastAngle + (deltaAngle / 2);

		// The offset in this frame
		Point localOffset;

		if (deltaAngle.Convert(radian) != 0.0) {
			// Calculate values based on how far it rotated that frame
			double rotationAdjustment = 2 * sin(deltaAngle / 2);
			localOffset = Point(((deltaBack/deltaAngle).getValue() + backOffset.getValue()) * rotationAdjustment, ((deltaRight/deltaAngle).getValue() + rightOffset.getValue()) * rotationAdjustment);
		} else {
			// Calculate the local offset then translate it to the global offset
			localOffset = Point(deltaBack, deltaRight);
		}

		// Rotate local offset
		double rotationCos = cos(averageOrientation - orientationOffset);
		double rotationSin = sin(averageOrientation - orientationOffset);

		// Calculate the rotation manually instead of using the vector class to increase speed an accuracy
		localOffset = Point(localOffset.getX().Convert(metre) * rotationCos + localOffset.getY().Convert(metre) * rotationSin, - localOffset.getX().Convert(metre) * rotationSin + localOffset.getY().Convert(metre) * rotationCos);
		
		// Set the global velocity vector
		this->setCurrentVelocity(Vector(&localOffset));

		// Add localOffset to the global offset
		lastPose.add(localOffset);
		lastPose.setAngle(fmod(angleDifference(currentAngle.Convert(radian), 0) + M_PI * 2, M_PI * 2));// + orientationOffset.getValue());

		// Update the position
		this->setPose(lastPose);
	}

	ThreeWheelOdom::~ThreeWheelOdom() {
	}
} // namespace Pronounce
