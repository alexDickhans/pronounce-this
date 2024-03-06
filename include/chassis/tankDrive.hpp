#pragma once

#include "api.h"
#include "hardwareDrivetrain.hpp"
#include "abstractTankDrivetrain.hpp"
#include "pros/motor_group.hpp"
#include "Logger/logger.hpp"

namespace Pronounce {
	class TankDrivetrain : public AbstractTankDrivetrain, public HardwareDrivetrain {
	private:
		static Logger *logger;
		pros::AbstractMotor& leftMotors;
		pros::AbstractMotor& rightMotors;
		QAngularVelocity maxMotorSpeed = 0.0;
	public:

		TankDrivetrain(const QLength &trackWidth, const QSpeed &maxSpeed, pros::AbstractMotor &leftMotors,
		               pros::AbstractMotor &rightMotors, const QAngularVelocity &maxMotorSpeed);

		QSpeed getSpeed() {
			return ((mean(leftMotors.get_actual_velocity_all()) + mean(rightMotors.get_actual_velocity_all())) /
			        2.0) / 600.0 * this->getMaxSpeed();
		}

		void skidSteerVelocity(QSpeed speed, double turn) {
			double power = speed.getValue();

			double turnSpeed = turn * this->getMaxSpeed().Convert(metre / second);

			double leftSpeed = power + turnSpeed;
			double rightSpeed = power - turnSpeed;

			double maxValue = max(abs(leftSpeed), abs(rightSpeed));

			if (maxValue > this->getMaxSpeed().getValue()) {
				leftSpeed = leftSpeed * (this->getMaxSpeed().getValue() / maxValue);
				rightSpeed = rightSpeed * (this->getMaxSpeed().getValue() / maxValue);
			}

			this->tankSteerVelocity(leftSpeed, rightSpeed);
		}

		void tankSteerVelocity(QSpeed leftSpeed, QSpeed rightSpeed) {
			Log("LeftVelocity: " + std::to_string(leftSpeed.Convert(inch / second)) + " RightVelocity: " +
			    std::to_string(rightSpeed.Convert(inch / second)));
			this->leftMotors.move_velocity(leftSpeed.getValue() * (maxMotorSpeed / this->getMaxSpeed()).getValue());
			this->rightMotors.move_velocity(rightSpeed.getValue() * (maxMotorSpeed / this->getMaxSpeed()).getValue());
		}

		void tankSteerVoltage(int32_t leftVoltage, int32_t rightVoltage) {
			this->leftMotors.move_voltage(leftVoltage);
			this->rightMotors.move_voltage(rightVoltage);

			Log("LeftVoltage: " + std::to_string(leftVoltage) + " RightVoltage: " + std::to_string(rightVoltage));
		}

		void reset() {
			Log("Reset Drivetrain Position");
			this->leftMotors.tare_position_all();
			this->rightMotors.tare_position_all();
		}

		QLength getDistanceSinceReset() override {
			leftMotors.set_encoder_units(pros::E_MOTOR_ENCODER_ROTATIONS);
			rightMotors.set_encoder_units(pros::E_MOTOR_ENCODER_ROTATIONS);
			return (
					       ((mean(leftMotors.get_position_all()) * revolution) *
					        (this->getMaxSpeed() / this->maxMotorSpeed))
					       + ((mean(rightMotors.get_position_all()) * revolution) *
					          (this->getMaxSpeed() / this->maxMotorSpeed))
			       ) / 2.0;
		}

		void setBrakeMode(pros::MotorBrake brakeMode) {
			leftMotors.set_brake_mode_all(brakeMode);
			rightMotors.set_brake_mode_all(brakeMode);
		}

		pros::MotorBrake getBrakeMode() {
			return leftMotors.get_brake_mode();
		}

		~TankDrivetrain() = default;
	};

	Logger *TankDrivetrain::logger{Logger::getInstance()};

	TankDrivetrain::TankDrivetrain(const QLength &trackWidth, const QSpeed &maxSpeed, pros::AbstractMotor &leftMotors,
	                               pros::AbstractMotor &rightMotors, const QAngularVelocity &maxMotorSpeed)
			: AbstractTankDrivetrain(trackWidth, maxSpeed), leftMotors(leftMotors), rightMotors(rightMotors),
			  maxMotorSpeed(maxMotorSpeed) {}
} // namespace Pronounce




