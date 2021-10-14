#pragma once

#include "api.h"
#include "odomWheel.hpp"

namespace Pronounce
{
    class MotorOdom : public OdomWheel {
    private:
        pros::Motor* motor;

        double radius;
    public:
        MotorOdom(pros::Motor* motor, double radius);

        void setMotor(pros::Motor* motor) {
            this->motor = motor;
        }

        pros::Motor* getMotor() {
            return this->motor;
        }

        void update();

        ~MotorOdom();
    };

} // namespace Pronounce
