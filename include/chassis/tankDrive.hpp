#pragma once

#include "api.h"
#include "hardwareDrivetrain.hpp"
#include "abstractTankDrivetrain.hpp"

namespace Pronounce {
	class TankDrivetrain : public AbstractTankDrivetrain, public HardwareDrivetrain {
	private:
		pros::Motor_Group& leftMotors;
		pros::Motor_Group& rightMotors;
		double maxMotorSpeed = 0.0;
	public:
		TankDrivetrain(QLength trackWidth, QSpeed maxSpeed, pros::Motor_Group& leftMotors, pros::Motor_Group& rightMotors, double maxMotorSpeed) : leftMotors(leftMotors), rightMotors(rightMotors), AbstractTankDrivetrain(trackWidth, maxSpeed) {

		}

		QSpeed getSpeed() {
			return ((leftMotors.get_actual_velocities().at(1) + rightMotors.get_actual_velocities().at(1)) / 2.0) * (this->getMaxSpeed()/maxMotorSpeed);
		}



		~TankDrivetrain() {

		}
	};
} // namespace Pronounce




