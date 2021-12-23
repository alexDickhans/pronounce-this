#pragma once

#include "api.h"
#include "drivetrain.hpp"
#include "utils/utils.hpp"

namespace Pronounce {
	class TankDrivetrain : public Drivetrain {
	private:
		double trackWidth;
	public:
		TankDrivetrain();
		TankDrivetrain(pros::Motor* frontLeft, pros::Motor* frontRight, pros::Motor* backLeft, pros::Motor* backRight, pros::Imu* imu);
		TankDrivetrain(pros::Motor* frontLeft, pros::Motor* frontRight, pros::Motor* backLeft, pros::Motor* backRight, pros::Imu* imu, double trackWidth);

		void skidSteerVelocity(double speed, double turn);

		void tankSteerVelocity(double leftSpeed, double rightSpeed);

		double getTrackWidth() {
			return this->trackWidth;
		}

		void setTrackWidth(double trackWidth) {
			this->trackWidth = trackWidth;
		}

		~TankDrivetrain();
	};
} // namespace Pronounce




