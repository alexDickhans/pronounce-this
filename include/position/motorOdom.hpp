#pragma once

#include "api.h"
#include "odomWheel.hpp"

namespace Pronounce
{
    class MotorOdom : public OdomWheel {
    private:
        pros::Motor* motor;

        double radius;
        double tuningFactor = 1.0;
    public:
        MotorOdom(pros::Motor* motor, double radius);

        double getTuringFactor() {
            return this->tuningFactor;
        }

        void setTuningFactor(double tuningFactor) {
            this->tuningFactor = tuningFactor;
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
