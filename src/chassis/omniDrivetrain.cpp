#include "drivetrain.hpp"
#include "omniDrivetrain.hpp"

namespace Pronounce {
    OmniDrivetrain::OmniDrivetrain(pros::Motor* frontLeft, pros::Motor* frontRight, pros::Motor* backLeft, pros::Motor* backRight, pros::Imu* imu) : Drivetrain(frontLeft, frontRight, backLeft, backRight, imu) {

    }

    OmniDrivetrain::~OmniDrivetrain() {
        
    }
} // namespace Pronounce
