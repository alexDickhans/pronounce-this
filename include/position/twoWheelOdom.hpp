#pragma once

#include "api.h"
#include "odomWheel.hpp"

namespace Pronounce
{
    class TwoWheelOdom
    {

    private:
        OdomWheel* forward;
        OdomWheel* horizontal;

        pros::Imu* imu;

        double offset;
    public:
        TwoWheelOdom(OdomWheel* forward, OdomWheel* horizontal, pros::Imu* imu);
        TwoWheelOdom(OdomWheel* forward, OdomWheel* horizontal, pros::Imu* imu, double offset);
        ~TwoWheelOdom();
    };
} // namespace Pronounce

