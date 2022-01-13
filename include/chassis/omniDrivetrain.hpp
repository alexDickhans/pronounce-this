#pragma once 

#include "api.h"
#include "drivetrain.hpp"
#include "utils/vector.hpp"

namespace Pronounce
{
    /**
     * Omnidirectional drive type, used for X-Drive/Mecanum drive
     */
    class OmniDrivetrain : public Drivetrain {
    private:
        
    public:
        OmniDrivetrain();
        OmniDrivetrain(pros::Motor* frontLeft, pros::Motor* frontRight, pros::Motor* backLeft, pros::Motor* backRight, pros::Imu* imu);

        virtual void setDriveVectorVelocity(Vector vector) {}
        virtual void setDriveVectorVelocity(Vector vector, double rotation) {}
        
        ~OmniDrivetrain();
    };
    
} // namespace Pronounce
