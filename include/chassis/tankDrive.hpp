#pragma once

#include "api.h"
#include "hardwareDrivetrain.hpp"
#include "abstractTankDrivetrain.hpp"

namespace Pronounce {
	class TankDrivetrain : public AbstractTankDrivetrain, public HardwareDrivetrain {
	private:
		pros::Motor_Group* leftMotors;
		pros::Motor_Group* rightMotors;
		double maxMotorSpeed = 0.0;
	public:
		TankDrivetrain(QLength trackWidth, QSpeed maxSpeed, pros::Motor_Group* leftMotors, pros::Motor_Group* rightMotors, double maxMotorSpeed) : leftMotors(leftMotors), rightMotors(rightMotors), AbstractTankDrivetrain(trackWidth, maxSpeed) {
			this->maxMotorSpeed = maxMotorSpeed;
		}

		QSpeed getSpeed() {
			return ((leftMotors->get_actual_velocities().at(1) + rightMotors->get_actual_velocities().at(1)) / 2.0) * (this->getMaxSpeed()/maxMotorSpeed);
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
			this->leftMotors->move_velocity(leftSpeed.getValue() * (maxMotorSpeed/this->getMaxSpeed()).getValue());
			this->rightMotors->move_velocity(rightSpeed.getValue() * (maxMotorSpeed/this->getMaxSpeed()).getValue());
		}

		void tankSteerVoltage(int32_t leftVoltage, int32_t rightVoltage) {
			this->leftMotors->move_voltage(leftVoltage);
			this->rightMotors->move_voltage(rightVoltage);

			std::cout << "LeftVoltage: " << leftVoltage << std::endl << "RightVoltage: " << rightVoltage << std::endl;
		}

		void reset() {
			for (int i = 0; i < this->leftMotors->size(); i++) {
				this->leftMotors->operator[](i).tare_position();
			}
			for (int i = 0; i < this->rightMotors->size(); i++) {
				this->rightMotors->operator[](i).tare_position();
			}
		}

		QLength getDistanceSinceReset() {
			leftMotors->set_encoder_units(pros::E_MOTOR_ENCODER_ROTATIONS);
			rightMotors->set_encoder_units(pros::E_MOTOR_ENCODER_ROTATIONS);
			return (((leftMotors->get_positions()[0] * 1_pi * 3.25_in) + (rightMotors->get_positions()[0] * 1_pi * 3.25_in)) / 2.0).getValue() * 360.0/600.0;
		}

		void setBrakeMode(pros::motor_brake_mode_e_t brakeMode) {
			leftMotors->set_brake_modes(brakeMode);
			rightMotors->set_brake_modes(brakeMode);
		}

		pros::motor_brake_mode_e_t getBrakeMode() {
			return leftMotors->operator[](0).get_brake_mode();
		}

		~TankDrivetrain() {

		}
	};
} // namespace Pronounce




