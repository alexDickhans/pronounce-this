#include "tankDrive.hpp"

namespace Pronounce {

	TankDrivetrain::TankDrivetrain(pros::Motor* frontLeft, pros::Motor* frontRight, pros::Motor* backLeft, pros::Motor* backRight, pros::Imu* imu) : Drivetrain(frontLeft, frontRight, backLeft, backRight, imu) {
	}

	TankDrivetrain::TankDrivetrain(pros::Motor* frontLeft, pros::Motor* frontRight, pros::Motor* backLeft, pros::Motor* backRight, pros::Imu* imu, double trackWidth) : Drivetrain(frontLeft, frontRight, backLeft, backRight, imu), AbstractTankDrivetrain(trackWidth) {
		
	}

	TankDrivetrain::TankDrivetrain(pros::Motor* frontLeft, pros::Motor* frontRight, pros::Motor* midLeft, pros::Motor* midRight, pros::Motor* backLeft, pros::Motor* backRight, pros::Imu* imu) : Drivetrain(frontLeft, frontRight, midLeft, midRight, backLeft, backRight, imu) {
		
	}

	TankDrivetrain::TankDrivetrain(pros::Motor* frontLeft, pros::Motor* frontRight, pros::Motor* midLeft, pros::Motor* midRight, pros::Motor* backLeft, pros::Motor* backRight, pros::Imu* imu, double trackWidth) : Drivetrain(frontLeft, frontRight, midLeft, midRight, backLeft, backRight, imu), AbstractTankDrivetrain(trackWidth) {
		
	}

	TankDrivetrain::~TankDrivetrain() {
	}
} // namespace Pronounce
