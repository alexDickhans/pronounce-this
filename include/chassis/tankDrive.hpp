#pragma once

#include "api.h"
#include "drivetrain.hpp"
#include "utils/utils.hpp"

namespace Pronounce {
	class TankDrivetrain : public Drivetrain {
	private:

	public:
		TankDrivetrain();
		TankDrivetrain(pros::Motor* frontLeft, pros::Motor* frontRight, pros::Motor* backLeft, pros::Motor* backRight, pros::Imu* imu);

		void skidSteerVelocity(double speed, double turn);

		void tankSteerVelocity(double leftSpeed, double rightSpeed);

		~TankDrivetrain();
	};
} // namespace Pronounce




