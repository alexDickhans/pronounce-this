#pragma once

#include "api.h"
#include "drivetrain.hpp"
#include "utils/motorGroup.hpp"

namespace Pronounce {
	class TankDrivetrain : public Drivetrain {
	private:
		double trackWidth;
	public:
		TankDrivetrain(pros::Motor* frontLeft, pros::Motor* frontRight, pros::Motor* backLeft, pros::Motor* backRight, pros::Imu* imu);
		TankDrivetrain(pros::Motor* frontLeft, pros::Motor* frontRight, pros::Motor* backLeft, pros::Motor* backRight, pros::Imu* imu, double trackWidth);

		void skidSteerVelocity(double speed, double turn) {
			this->getLeftMotors().move_velocity(speed - turn * this->trackWidth / 2);
			this->getRightMotors().move_velocity(speed + turn * this->trackWidth / 2);
		}

		void tankSteerVelocity(double leftSpeed, double rightSpeed) {
			printf("leftSpeed: %f, leftSpeed: %f \n", leftSpeed, rightSpeed);
			this->getLeftMotors().move_velocity(leftSpeed);
			this->getRightMotors().move_velocity(rightSpeed);
		}

		double getTrackWidth() {
			return this->trackWidth;
		}

		void setTrackWidth(double trackWidth) {
			this->trackWidth = trackWidth;
		}

		~TankDrivetrain();
	};
} // namespace Pronounce




