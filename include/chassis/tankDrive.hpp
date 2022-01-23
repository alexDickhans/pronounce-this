#pragma once

#include "api.h"
#include "drivetrain.hpp"
#include "abstractTankDrivetrain.hpp"
#include "utils/motorGroup.hpp"

namespace Pronounce {
	class TankDrivetrain : public AbstractTankDrivetrain, public Drivetrain {
	private:

	public:
		TankDrivetrain(pros::Motor* frontLeft, pros::Motor* frontRight, pros::Motor* backLeft, pros::Motor* backRight, pros::Imu* imu);
		TankDrivetrain(pros::Motor* frontLeft, pros::Motor* frontRight, pros::Motor* backLeft, pros::Motor* backRight, pros::Imu* imu, double trackWidth);
		TankDrivetrain(pros::Motor* frontLeft, pros::Motor* frontRight, pros::Motor* midLeft, pros::Motor* midRight, pros::Motor* backLeft, pros::Motor* backRight, pros::Imu* imu);
		TankDrivetrain(pros::Motor* frontLeft, pros::Motor* frontRight, pros::Motor* midLeft, pros::Motor* midRight, pros::Motor* backLeft, pros::Motor* backRight, pros::Imu* imu, double trackWidth);

		void skidSteerVelocity(double speed, double turn) {
			this->getLeftMotors().move_velocity(speed + turn);
			this->getRightMotors().move_velocity(speed - turn);
		}

		void tankSteerVelocity(double leftSpeed, double rightSpeed) {
			// std::cout << "2Left speed: " << leftSpeed << " Right speed: " << rightSpeed << std::endl;
			this->getLeftMotors().move_velocity(leftSpeed);
			this->getRightMotors().move_velocity(rightSpeed);
		}

		~TankDrivetrain();
	};
} // namespace Pronounce




