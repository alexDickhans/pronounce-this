#include "drivetrain.hpp"

namespace Pronounce {

	Drivetrain::Drivetrain() {
		
	}

    Drivetrain::Drivetrain(pros::Motor* frontLeft, pros::Motor* frontRight, pros::Motor* backLeft, pros::Motor* backRight, pros::Imu* imu) {
		std::vector<pros::Motor*> leftMotors;
		std::vector<pros::Motor*> rightMotors;
		leftMotors.emplace_back(frontLeft);
		leftMotors.emplace_back(backLeft);
		rightMotors.emplace_back(frontRight);
		rightMotors.emplace_back(backRight);
		this->leftMotors = MotorGroup(leftMotors);
		this->rightMotors = MotorGroup(rightMotors);
        this->imu = imu;
    }

	Drivetrain::Drivetrain(MotorGroup leftMotors, MotorGroup rightMotors, pros::Imu* imu) {
		this->leftMotors = leftMotors;
		this->rightMotors = rightMotors;
		this->imu = imu;
	}

    double Drivetrain::getTemp() {
        double total = this->leftMotors.get_temperature() +
						this->rightMotors.get_temperature();
        return total / 2.0;
    }
    
    double Drivetrain::getSpeed() {
        double total = this->leftMotors.get_actual_velocity() + 
						this->rightMotors.get_actual_velocity();
        return total / 2.0;
    }

} // namespace Pronounce

