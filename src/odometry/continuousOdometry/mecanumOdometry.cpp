#include "mecanumOdometry.hpp"

namespace Pronounce {
    MecanumOdometry::MecanumOdometry(/* args */) {
    }

    MecanumOdometry::MecanumOdometry(double xOffset, double yOffset) {
        this->xOffset = xOffset;
        this->yOffset = yOffset;
    }

    MecanumOdometry::MecanumOdometry(OdomWheel* wheel1, OdomWheel* wheel2, OdomWheel* wheel3, OdomWheel* wheel4, pros::Imu* imu, double xOffset, double yOffset) : MecanumOdometry(xOffset, yOffset) {
        this->wheel1 = wheel1;
        this->wheel2 = wheel2;
        this->wheel3 = wheel3;
        this->wheel4 = wheel4;
        this->imu = imu;
    }

    void MecanumOdometry::update() {
        Pose2D* lastPose = this->getPose();
        Pose2D* newPose = new Pose2D();
        newPose->operator=(lastPose);

        // Update the wheel positions
        wheel1->update();
        wheel2->update();
        wheel3->update();
        wheel4->update();

        Angle angleChange = 0.0;
        Angle lastAngle = lastPose->getAngle();

        double deltaWheel1 = wheel1->getChange();
        double deltaWheel2 = wheel2->getChange();
        double deltaWheel3 = wheel3->getChange();
        double deltaWheel4 = wheel4->getChange();

        if (useImu && imu != nullptr) {
            if (imu->is_calibrating()) {
                return;
            }

            // Use the imu to determine the orientation
			Angle imuAngle = toRadians(imu->get_rotation());
            newPose->setAngle(imuAngle + this->getResetPose()->getAngle());

            angleChange = newPose->getAngle() - lastPose->getAngle();

        }
        else {
            // Use the wheel encoders to determine orientation

            // Calculate the orientation since last reset
            Angle currentAngle = (-wheel1->getPosition() + wheel2->getPosition() - wheel3->getPosition() + wheel4->getPosition()) / (4 * xOffset, yOffset);

            angleChange = currentAngle - lastAngle;

            newPose->setAngle(currentAngle);
        }

        Angle averageOrientation = lastAngle + angleChange / 2;

        Angle orientation = newPose->getAngle();


        Vector localOffset(
            new Point(
                -(-deltaWheel1 + deltaWheel2 + deltaWheel3 - deltaWheel4) / 4,
                (deltaWheel1 + deltaWheel2 + deltaWheel3 + deltaWheel4) / 4));

        localOffset.rotate(-averageOrientation.getValue());
        newPose->add(localOffset.getCartesian());

        // Update the position
        this->setPose(newPose);
    }

    MecanumOdometry::~MecanumOdometry() {
    }
}; // namespace Pronounce