#pragma once

#include "api.h"

namespace Pronounce
{
    class TwoWheelOdom
    {

    private:
        pros::Rotation* forward;
        pros::Rotation* horizontal;

        pros::Imu* imu;
    public:
        TwoWheelOdom(pros::Rotation* forward, pros::Rotation* horizontal, pros::Imu* imu);
        ~TwoWheelOdom();
    };
    
    TwoWheelOdom::TwoWheelOdom(pros::Rotation* forward, pros::Rotation* horizontal, pros::Imu* imu) {
        this->forward = forward;
        this->horizontal = horizontal;
        this->imu = imu;

    }
    
    TwoWheelOdom::~TwoWheelOdom()
    {
    }
    
} // namespace Pronounce

