#pragma once 

#include "api.h"
#include "drivetrain.hpp"

namespace Pronounce
{
    /**
     * Omnidirectional drive type, used for X-Drive/Mecanum drive
     */
    class OmniDrivetrain : protected Drivetrain
    {
    protected:
        /* data */
    public:
        OmniDrivetrain(pros::Motor* frontLeft, pros::Motor* frontRight, pros::Motor* backLeft, pros::Motor* backRight, pros::Imu* imu);
        
        
        
        ~OmniDrivetrain();
    };
    
} // namespace Pronounce
