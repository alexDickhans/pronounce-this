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

		double getSpeed() override {
			return (this->getLeftMotors().get_target_velocity() + this->getRightMotors().get_target_velocity()) / 2;
		}

		void skidSteerVelocity(double speed, double turn) {
			this->getLeftMotors().move_velocity(speed + turn);
			this->getRightMotors().move_velocity(speed - turn);
		}

		void tankSteerVelocity(double leftSpeed, double rightSpeed) {
			this->getLeftMotors().move_velocity(leftSpeed);
			this->getRightMotors().move_velocity(rightSpeed);
		}

		~TankDrivetrain();
	};
} // namespace Pronounce




