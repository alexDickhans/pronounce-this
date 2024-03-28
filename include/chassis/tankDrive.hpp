#pragma once

#include "api.h"
#include "hardwareDrivetrain.hpp"
#include "abstractTankDrivetrain.hpp"
#include "pros/motor_group.hpp"
#include "Logger/logger.hpp"

namespace Pronounce {
	class TankDrivetrain : public AbstractTankDrivetrain, public HardwareDrivetrain {
	private:
		pros::AbstractMotor& leftMotors;
		pros::AbstractMotor& rightMotors;
		QAngularVelocity maxMotorSpeed = 0.0;
	public:

		TankDrivetrain(const QLength &trackWidth, const QVelocity &maxSpeed, pros::AbstractMotor &leftMotors,
		               pros::AbstractMotor &rightMotors, const QAngularVelocity &maxMotorSpeed);

		QVelocity getSpeed() final {
			return ((mean(leftMotors.get_actual_velocity_all()) + mean(rightMotors.get_actual_velocity_all())) /
			        2.0) / 600.0 * this->getMaxSpeed();
		}

		void tankSteerVoltage(int32_t leftVoltage, int32_t rightVoltage) override {
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
			leftMotors.set_encoder_units_all(pros::E_MOTOR_ENCODER_ROTATIONS);
			rightMotors.set_encoder_units_all(pros::E_MOTOR_ENCODER_ROTATIONS);
			auto result = (
					                ((mean(leftMotors.get_position_all()) * revolution) *
					                 (this->getMaxSpeed() / this->maxMotorSpeed))
					                + ((mean(rightMotors.get_position_all()) * revolution) *
					                   (this->getMaxSpeed() / this->maxMotorSpeed))
			                ) / 2.0;

			Log(std::to_string(result.Convert(inch)));
			return result;
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

	TankDrivetrain::TankDrivetrain(const QLength &trackWidth, const QVelocity &maxSpeed, pros::AbstractMotor &leftMotors,
	                               pros::AbstractMotor &rightMotors, const QAngularVelocity &maxMotorSpeed)
			: AbstractTankDrivetrain(trackWidth, maxSpeed), leftMotors(leftMotors), rightMotors(rightMotors),
			  maxMotorSpeed(maxMotorSpeed) {}
} // namespace Pronounce




