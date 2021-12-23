#include "tankDrive.hpp"

namespace Pronounce {

	TankDrivetrain::TankDrivetrain(pros::Motor* frontLeft, pros::Motor* frontRight, pros::Motor* backLeft, pros::Motor* backRight, pros::Imu* imu) : Drivetrain(frontLeft, frontRight, backLeft, backRight, imu) {
		
	}

	void TankDrivetrain::skidSteerVelocity(double speed, double turn) {
		this->frontLeft->move_velocity(speed + turn);
		this->frontRight->move_velocity(speed - turn);
		this->backLeft->move_velocity(speed + turn);
		this->backRight->move_velocity(speed - turn);
	}

	void TankDrivetrain::tankSteerVelocity(double leftSpeed, double rightSpeed) {
		this->frontLeft->move_velocity(leftSpeed);
		this->frontRight->move_velocity(rightSpeed);
		this->backLeft->move_velocity(leftSpeed);
		this->backRight->move_velocity(rightSpeed);
	}

	TankDrivetrain::~TankDrivetrain() {
	}
} // namespace Pronounce
