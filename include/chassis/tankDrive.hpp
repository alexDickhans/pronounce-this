#pragma once

#include <algorithm>
#include "drivetrain.hpp"
#include "position/tankOdom.hpp"
#include "pid/pid.hpp"
#include "okapi/api.hpp"
#include "utils/utils.hpp"

namespace Pronounce {
    class TankDrivetrain : public Drivetrain {
    private:
        bool enabled = false;

        TankOdom* tankOdom;

        Position* targetPosition;
        Position* startingPosition;

        PID* turnPid;
        PID* movePid;

        double angle;
        double prevAngle;

        double nullRotationDistance = 10.0;
        double maxVoltage = 127.0;

        double speedThreshhold = 2.0;
        double errorThreshhold = 0.5;

        double turnThreshhold = 2.0;
        double turnErrorThreshhold = 5.0;
    public:
        TankDrivetrain(pros::Motor* frontLeft, pros::Motor* frontRight, pros::Motor* backLeft, pros::Motor* backRight, pros::Imu* imu);
        void reset();

        void update();

        void waitForStop();

        bool getStopped();

        void setAngle(double angle) {
            this->angle = angle;
        }

        double getMaxVoltage() {
            return this->maxVoltage;
        }

        void setMaxVoltage(double maxVoltage) {
            this->maxVoltage = maxVoltage;
        }

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
            this->imu->set_rotation(position->getTheta());
            angle = NAN;
        }

        void setPosition(Position* position) {
            this->tankOdom->setPosition(position);
        }

        Position* getTargetPosition() {
            return targetPosition;
        }

        void setTargetPosition(Position* targetPosition) {
            this->startingPosition = this->getPosition();
            angle = NAN;
            this->targetPosition = targetPosition;
        }

        ~TankDrivetrain();
    };
} // namespace Pronounce




