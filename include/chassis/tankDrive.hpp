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
			this->maxMotorSpeed = maxMotorSpeed;
		}

		QSpeed getSpeed() {
			return ((leftMotors.get_actual_velocities().at(1) + rightMotors.get_actual_velocities().at(1)) / 2.0) * (this->getMaxSpeed()/maxMotorSpeed);
		}

		void skidSteerVelocity(QSpeed speed, double turn) {
			double power = speed.getValue();

			double turnSpeed = turn * this->getMaxSpeed().Convert(metre/second);

			double leftSpeed = power + turnSpeed;
			double rightSpeed = power - turnSpeed;

			double maxValue = max(abs(leftSpeed), abs(rightSpeed));

			if (maxValue > this->getMaxSpeed().getValue()) {
				leftSpeed = leftSpeed * (this->getMaxSpeed().getValue()/maxValue);
				rightSpeed = rightSpeed * (this->getMaxSpeed().getValue()/maxValue);
			}

			this->tankSteerVelocity(leftSpeed, rightSpeed);
		}

		void tankSteerVelocity(QSpeed leftSpeed, QSpeed rightSpeed) {
			std::cout << "LeftVelocity: " << leftSpeed.Convert(inch/second) << std::endl << "RightVelocity: " << rightSpeed.Convert(inch/second) << std::endl;
			this->leftMotors.move_velocity(leftSpeed.getValue() * (maxMotorSpeed/this->getMaxSpeed()).getValue());
			this->rightMotors.move_velocity(rightSpeed.getValue() * (maxMotorSpeed/this->getMaxSpeed()).getValue());
		}

		void tankSteerVoltage(int16_t leftVoltage, int16_t rightVoltage) {
			this->leftMotors.move_voltage(leftVoltage);
			this->rightMotors.move_voltage(rightVoltage);
		}

		~TankDrivetrain() {

		}
	};
} // namespace Pronounce




