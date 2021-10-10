#include "tankOdom.hpp"

namespace Pronounce
{
    TankOdom::TankOdom(OdomWheel* leftPivot, OdomWheel* rightPivot, pros::Imu* imu) {
        this->leftPivot = leftPivot;
        this->rightPivot = rightPivot;
        this->imu = imu;
    }

    void TankOdom::update() {
        double average = (leftPivot->getChange() + rightPivot->getChange()) / 2;
        double angle = toRadians(imu->get_rotation());

        double x1 = 0;
        double y1 = average;

        double x2 = (x1 * sin(angle)) - (y1 * cos(angle));
        double y2 = (x1 * sin(angle)) + (y1 * cos(angle));

        this->position->setX(this->position->getX() + x2);
        this->position->setY(this->position->getY() + y2);
    }

    TankOdom::~TankOdom() {
    }
} // namespace Pronounce
