#pragma once

#include "api.h"
#include "odomWheel.hpp"

// TODO: Add docstrings
// TODO: Add comments
// TODO: make sure code is up to date

namespace Pronounce
{
    class MotorOdom : public OdomWheel {
    private:
        std::shared_ptr<pros::Motor> motor;
    public:
        MotorOdom(std::shared_ptr<pros::Motor> motor, QLength radius);

        void reset() {
            motor->tare_position();
        }

        std::shared_ptr<pros::Motor> getMotor() {
            return this->motor;
        }

        void update();

        ~MotorOdom();
    };

} // namespace Pronounce
