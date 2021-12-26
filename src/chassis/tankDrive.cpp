#include "tankDrive.hpp"

namespace Pronounce {

	TankDrivetrain::TankDrivetrain(pros::Motor* frontLeft, pros::Motor* frontRight, pros::Motor* backLeft, pros::Motor* backRight, pros::Imu* imu) : Drivetrain(frontLeft, frontRight, backLeft, backRight, imu) {
	}

	TankDrivetrain::TankDrivetrain(pros::Motor* frontLeft, pros::Motor* frontRight, pros::Motor* backLeft, pros::Motor* backRight, pros::Imu* imu, double trackWidth) : Drivetrain(frontLeft, frontRight, backLeft, backRight, imu) {
		this->trackWidth = trackWidth;
	}

	void TankDrivetrain::skidSteerVelocity(double speed, double turn) {
		this->getLeftMotors().move_velocity(speed - turn * this->trackWidth / 2);
		this->getRightMotors().move_velocity(speed + turn * this->trackWidth / 2);
	}

	void TankDrivetrain::tankSteerVelocity(double leftSpeed, double rightSpeed) {
		this->getLeftMotors().move_velocity(leftSpeed);
		this->getRightMotors().move_velocity(rightSpeed);
	}

	TankDrivetrain::~TankDrivetrain() {
	}
} // namespace Pronounce
