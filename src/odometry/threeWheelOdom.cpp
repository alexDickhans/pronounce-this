#include "threeWheelOdom.hpp"

namespace Pronounce {
    ThreeWheelOdom::ThreeWheelOdom(/* args */) : Odometry() {
    }

    ThreeWheelOdom::ThreeWheelOdom(OdomWheel* leftWheel, OdomWheel* rightWheel, OdomWheel* backWheel,pros::Imu* imu) : Odometry() {
        this->leftWheel = leftWheel;
        this->rightWheel = rightWheel;
        this->backWheel = backWheel;
        this->imu = imu;
    }

    void ThreeWheelOdom::update() {
        //Plane adjustment toggle
        bool planeAdjustment = true;

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
        double currentAngle = this->getResetPosition()->getTheta() + (leftWheel->getPosition() - rightWheel->getPosition()) / (rightOffset + rightOffset);
        double angleChange = angleDifference(currentAngle, lastAngle); // fmod(currentAngle + M_PI * 2, M_PI * 2) - fmod(lastAngle + M_PI * 2, M_PI * 2);
        double averageOrientation = lastAngle + (angleChange / 2);

        // Calculate the local offset then translate it to the global offset
        Vector localOffset = Vector(new Point(deltaBack + (angleChange * backOffset), deltaRight + (angleChange * rightOffset)));
        
        //Calculate the difference between the plane the robot is on and the flat plane and change that
        if (planeAdjustment){
            //Formula doesn't work if pitch or roll are 0
            if (this->imu->getRoll() != 0 && this->imu->getPitch() != 0){
                localOffset.scale(cos(asin(sin(abs(this->imu->getRoll()))*sin(abs(this->imu->getPitch())))));
            //Just Pitch without roll
            }else (this->imu->getPitch() != 0){
                localOffset.scale(cos(this->imu->getPitch()));
            }
        }
        // Rotate vector
        localOffset.rotate(-averageOrientation);

        // Add localOffset to the global offset
        lastPosition->add(localOffset.getCartesian());
        lastPosition->setTheta(fmod(currentAngle + M_PI * 2, M_PI * 2));

        // Print last position
        printf("Last position: %f, %f, %f\n", lastPosition->getX(), lastPosition->getY(), lastPosition->getTheta());

        // Update the position
        this->setPosition(lastPosition);
    }

    ThreeWheelOdom::~ThreeWheelOdom() {
    }
} // namespace Pronounce
