#pragma once

#include "api.h"
#include "odomWheel.hpp"
#include "utils/position.hpp"

namespace Pronounce
{
    class TwoWheelOdom {

    private:
        OdomWheel* vertical;
        OdomWheel* horizontal;

        pros::Imu* imu;

        double offset;

        Position* position;
    public:
        TwoWheelOdom(OdomWheel* vertical, OdomWheel* horizontal, pros::Imu* imu);
        TwoWheelOdom(OdomWheel* vertical, OdomWheel* horizontal, pros::Imu* imu, double offset);

        void update();

        ~TwoWheelOdom();
    };
} // namespace Pronounce

