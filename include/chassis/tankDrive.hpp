#pragma once

#include "drivetrain.hpp"
#include "position/tankOdom.hpp"

namespace Pronounce {
    class TankDrivetrain : public Drivetrain {
    private:
        TankOdom tankOdom;

        Position* targetPosition;
    public:
        TankDrivetrain(pros::Motor* frontLeft, pros::Motor* frontRight, pros::Motor* backLeft, pros::Motor* backRight, pros::Imu* imu);

        Position* getPosition() {
            return this->tankOdom.getPosition();
        }

        void setPosition(Position position) {
            this->setPosition(position);
        }

        Position* getTargetPosition() {
            return targetPosition;
        }

        void setTargetPosition(Position* targetPosition) {
            this->targetPosition = targetPosition;
        }

        void update();

        ~TankDrivetrain();
    };
} // namespace Pronounce




TankDrivetrain::TankDrivetrain(pros::Motor* frontLeft, pros::Motor* frontRight, pros::Motor* backLeft, pros::Motor* backRight, pros::Imu* imu) {
}

void update();

TankDrivetrain::~TankDrivetrain() {
}
