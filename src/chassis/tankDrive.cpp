#include "tankDrive.hpp"

namespace Pronounce {

	TankDrivetrain::TankDrivetrain(pros::Motor* frontLeft, pros::Motor* frontRight, pros::Motor* backLeft, pros::Motor* backRight, pros::Imu* imu) : Drivetrain(frontLeft, frontRight, backLeft, backRight, imu) {
	}

	TankDrivetrain::TankDrivetrain(pros::Motor* frontLeft, pros::Motor* frontRight, pros::Motor* backLeft, pros::Motor* backRight, pros::Imu* imu, double trackWidth) : Drivetrain(frontLeft, frontRight, backLeft, backRight, imu) {
		this->trackWidth = trackWidth;
	}

	TankDrivetrain::~TankDrivetrain() {
	}
} // namespace Pronounce
