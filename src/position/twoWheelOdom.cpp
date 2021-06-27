#include "twoWheelOdom.hpp"

namespace Pronounce
{
    
    TwoWheelOdom::TwoWheelOdom(OdomWheel* forward, OdomWheel* horizontal, pros::Imu* imu, double offset) {
        this->forward = forward;
        this->horizontal = horizontal;
        this->imu = imu;
        this->offset = offset;
    }
    
    TwoWheelOdom::~TwoWheelOdom() {
    }
} // namespace Pronounce
