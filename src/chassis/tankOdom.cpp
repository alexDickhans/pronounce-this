#include "tankDrive.hpp"

namespace Pronounce {

    TankDrivetrain::TankDrivetrain(pros::Motor* frontLeft, pros::Motor* frontRight, pros::Motor* backLeft, pros::Motor* backRight, pros::Imu* imu) {
    }

    void TankDrivetrain::update() {
        Position* currentPosition = tankOdom.getPosition();

        double xDiff = this->targetPosition->getX() - currentPosition->getX();
        double yDiff = this->targetPosition->getY() - currentPosition->getY();
        
    }

    TankDrivetrain::~TankDrivetrain() {
    }
} // namespace Pronounce
