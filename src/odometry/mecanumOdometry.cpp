#include "mecanumOdometry.hpp"

namespace Pronounce {
    MecanumOdometry::MecanumOdometry(/* args */) {
    }

    MecanumOdometry::MecanumOdometry(double wheelRadius, double xOffset, double yOffset) {
        this->wheelRadius = wheelRadius;
        this->xOffset = xOffset;
        this->yOffset = yOffset;
    }

    MecanumOdometry::MecanumOdometry(OdomWheel* wheel1, OdomWheel* wheel2, OdomWheel* wheel3, OdomWheel* wheel4, pros::Imu* imu, double wheelRadius, double xOffset, double yOffset) : MecanumOdometry(wheelRadius, xOffset, yOffset) {
        this->wheel1 = wheel1;
        this->wheel2 = wheel2;
        this->wheel3 = wheel3;
        this->wheel4 = wheel4;
        this->imu = imu;
    }

    void MecanumOdometry::update() {
        Position* lastPosition = this->getPosition();
        Position* newPosition = new Position();
        newPosition->operator=(lastPosition);

        // Update the wheel positions
        wheel1->update();
        wheel2->update();
        wheel3->update();
        wheel4->update();

        double angleChange = 0;
        double lastAngle = lastPosition->getTheta();

        if (useImu && imu != nullptr) {
            // Use the imu to determine the orientation
            newPosition->setTheta(toRadians(imu->get_rotation()));

            angleChange = newPosition->getTheta() - lastPosition->getTheta();

        } else {
            // Use the wheel encoders to determine orientation

            // Calculate the orientation since last reset
            double currentAngle = (-wheel1->getPosition() + wheel2->getPosition() - wheel3->getPosition() + wheel4->getPosition()) / (4 * xOffset, yOffset);

            angleChange = currentAngle - lastAngle;

            newPosition->setTheta(currentAngle);
        }

        double averageOrientation = lastAngle + angleChange/2;

        double orientation = newPosition->getTheta();

        Vector localOffset = Vector(
            new Point(
                (wheel1->getChange() + wheel2->getChange() + wheel3->getChange() + wheel4->getChange()) / 4,
                (-wheel1->getChange() + wheel2->getChange() + wheel3->getChange() - wheel4->getChange()) / 4));
        localOffset.rotate(averageOrientation);
        newPosition->add(localOffset.getCartesian());

        // Update the position
        this->setPosition(newPosition);

    }

    MecanumOdometry::~MecanumOdometry() {
    }
}; // namespace Pronounce