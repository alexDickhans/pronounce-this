#pragma once

#include "api.h"
#include "odomWheel.hpp"

namespace Pronounce
{
    class MotorOdom : public OdomWheel {
    private:
        pros::Motor* motor;
    public:
        MotorOdom(pros::Motor* motor, double radius);

        void reset() {
            motor->tare_position();
        }

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
