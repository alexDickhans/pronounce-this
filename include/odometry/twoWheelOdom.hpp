#pragma once

#include "api.h"
#include "position/odomWheel.hpp"
#include "utils/position.hpp"

namespace Pronounce
{
    /**
     * @brief Two wheel odometry with IMU and 2 perpendicular odom wheels
     * 
     * @deprecated Implementation not tested
     * 
     */
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

