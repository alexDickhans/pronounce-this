#include "tankDrive.hpp"

namespace Pronounce {

	TankDrivetrain::TankDrivetrain(pros::Motor* frontLeft, pros::Motor* frontRight, pros::Motor* backLeft, pros::Motor* backRight, pros::Imu* imu) : Drivetrain(frontLeft, frontRight, backLeft, backRight, imu) {
	}

	TankDrivetrain::TankDrivetrain(pros::Motor* frontLeft, pros::Motor* frontRight, pros::Motor* backLeft, pros::Motor* backRight, pros::Imu* imu, double trackWidth) : Drivetrain(frontLeft, frontRight, backLeft, backRight, imu) {
		this->trackWidth = trackWidth;
	}

	TankDrivetrain::TankDrivetrain(pros::Motor* frontLeft, pros::Motor* frontRight, pros::Motor* midLeft, pros::Motor* midRight, pros::Motor* backLeft, pros::Motor* backRight, pros::Imu* imu) {
		std::vector<pros::Motor*> leftMotors;
		std::vector<pros::Motor*> rightMotors;
		leftMotors.emplace_back(frontLeft);
		leftMotors.emplace_back(midLeft);
		leftMotors.emplace_back(backLeft);
		rightMotors.emplace_back(frontRight);
		rightMotors.emplace_back(midRight);
		rightMotors.emplace_back(backRight);
		this->setLeftMotors(MotorGroup(leftMotors));
		this->setRightMotors(MotorGroup(rightMotors));
        this->setImu(imu);
	}

	TankDrivetrain::TankDrivetrain(pros::Motor* frontLeft, pros::Motor* frontRight, pros::Motor* midLeft, pros::Motor* midRight, pros::Motor* backLeft, pros::Motor* backRight, pros::Imu* imu, double trackWidth) {
		std::vector<pros::Motor*> leftMotors;
		std::vector<pros::Motor*> rightMotors;
		leftMotors.emplace_back(frontLeft);
		leftMotors.emplace_back(midLeft);
		leftMotors.emplace_back(backLeft);
		rightMotors.emplace_back(frontRight);
		rightMotors.emplace_back(midRight);
		rightMotors.emplace_back(backRight);
		this->setLeftMotors(MotorGroup(leftMotors));
		this->setRightMotors(MotorGroup(rightMotors));
        this->setImu(imu);
		this->trackWidth = trackWidth;
	}

	TankDrivetrain::~TankDrivetrain() {
	}
} // namespace Pronounce
