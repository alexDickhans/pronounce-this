#pragma once

#include "drivetrain.hpp"
#include "position/tankOdom.hpp"
#include "pid/pid.hpp"

namespace Pronounce {
    class TankDrivetrain : public Drivetrain {
    private:
        bool enabled = false;

        TankOdom* tankOdom;

        Position* targetPosition;
        Position* startingPosition;

        PID* turnPid;
        PID* movePid;

        double nullRotationDistance = 10.0;
    public:
        TankDrivetrain(pros::Motor* frontLeft, pros::Motor* frontRight, pros::Motor* backLeft, pros::Motor* backRight, pros::Imu* imu);

        void reset();

        void update();

        bool getEnabled() {
            return enabled;
        }

        void setEnabled(bool enabled) {
            this->enabled = enabled;
        }

        TankOdom* getTankOdom() {
            return tankOdom;
        }

        void setTankOdom(TankOdom* tankOdom) {
            this->tankOdom = tankOdom;
        }

        PID* getTurnPid() {
            return turnPid;
        }

        void setTurnPid(PID* turnPid) {
            this->turnPid = turnPid;
        }

        PID* getMovePid() {
            return movePid;
        }

        void setMovePid(PID* movePid) {
            this->movePid = movePid;
        }

        Position* getPosition() {
            return this->tankOdom->getPosition();
        }

        void setStartingPosition(Position* position) {
            this->tankOdom->setPosition(position);
            this->startingPosition = startingPosition;
        }

        void setPosition(Position* position) {
            this->tankOdom->setPosition(position);
        }

        Position* getTargetPosition() {
            return targetPosition;
        }

        void setTargetPosition(Position* targetPosition) {
            this->setStartingPosition(this->getPosition());
            this->targetPosition = targetPosition;
        }

        ~TankDrivetrain();
    };
} // namespace Pronounce




