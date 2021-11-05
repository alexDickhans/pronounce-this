#pragma once

#include "api.h"
#include "position/odomWheel.hpp"
#include "utils/position.hpp"

namespace Pronounce
{
    class TwoWheelOdom {

    private:
        OdomWheel* vertical;
        OdomWheel* horizontal;

        pros::Imu* imu;

        double angleOffset;
        double verticalOffset;
        double horizontalOffset;

        Position* position;
    public:
        TwoWheelOdom(OdomWheel* vertical, OdomWheel* horizontal, pros::Imu* imu);
        TwoWheelOdom(OdomWheel* vertical, OdomWheel* horizontal, pros::Imu* imu, double angleOffset);

        void update();

        ~TwoWheelOdom();
    };
} // namespace Pronounce

