#include "drivetrain.hpp"

namespace Pronounce {

    Drivetrain::Drivetrain(pros::Motor* frontLeft, pros::Motor* frontRight, pros::Motor* backLeft, pros::Motor* backRight, pros::Imu* imu){
        this->frontLeft = frontLeft;
        this->frontRight = frontRight;
        this->backLeft = backLeft;
        this->backRight = backRight;
        this->imu = imu;
    }

    double Drivetrain::getTemp() {
        double total = this->frontLeft->get_temperature() +
            this->frontRight->get_temperature() +
            this->backLeft->get_temperature() +
            this->backRight->get_temperature();
        return total / 4;
    }
    
    double Drivetrain::getSpeed() {
        double total = this->frontLeft->get_voltage() +
            this->frontRight->get_voltage() +
            this->backLeft->get_voltage() +
            this->backRight->get_voltage();
        return total / 4;
    }

} // namespace Pronounce

