#include "threeWheelOdom.hpp"

namespace Pronounce {
    ThreeWheelOdom::ThreeWheelOdom(/* args */) : Odometry() {
		this->imu = nullptr;
		this->reset(new Position());
    }

	ThreeWheelOdom::ThreeWheelOdom(OdomWheel* leftWheel, OdomWheel* rightWheel, OdomWheel* backWheel) : Odometry() {
		this->leftWheel = leftWheel;
		this->rightWheel = rightWheel;
		this->backWheel = backWheel;
		this->imu = nullptr;
		this->reset(new Position());
	}

    ThreeWheelOdom::ThreeWheelOdom(OdomWheel* leftWheel, OdomWheel* rightWheel, OdomWheel* backWheel,pros::Imu* imu) : Odometry() {
        this->leftWheel = leftWheel;
        this->rightWheel = rightWheel;
        this->backWheel = backWheel;
        this->imu = imu;
		this->reset(new Position());
    }

    void ThreeWheelOdom::update() {
        if (planeAdjustment && imu != nullptr){
            planeAdjustment = false;
        }

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
		} else {
        	currentAngle = this->getResetPosition()->getTheta() + (leftWheel->getPosition() - rightWheel->getPosition()) / (leftOffset + rightOffset);
		}

        double angleChange = angleDifference(currentAngle, lastAngle);
        double averageOrientation = lastAngle + (angleChange / 2);

        // Calculate the local offset then translate it to the global offset
        Vector localOffset = Vector(new Point(deltaBack + (angleChange * backOffset), deltaRight + (angleChange * rightOffset)));
        
        //Calculate the difference between the plane the robot is on and the flat plane and change that
        if (planeAdjustment){
            //Formula doesn't work if pitch or roll are 0
            if (this->imu->get_roll() != 0 && this->imu->get_pitch() != 0) {
                localOffset.scale(cos(asin(sin(abs(this->imu->get_roll()))*sin(abs(this->imu->get_pitch())))));
            //Just Pitch without roll
            } else if (this->imu->get_pitch() != 0) {
                localOffset.scale(cos(this->imu->get_pitch()));
            }
        }
        // Rotate vector
        localOffset.rotate(-averageOrientation);

        // Add localOffset to the global offset
        lastPosition->add(localOffset.getCartesian());
        lastPosition->setTheta(fmod(currentAngle + M_PI * 2, M_PI * 2));

		if (localOffset.getMagnitude() > maxMovement) {
			return;
		}

        // Update the position
        this->setPosition(lastPosition);
    }

    ThreeWheelOdom::~ThreeWheelOdom() {
    }
} // namespace Pronounce
