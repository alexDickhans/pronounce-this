#pragma once

#include "api.h"
#include "pros/motor_group.hpp"
#include "logger/logger.hpp"

namespace Pronounce {
	class TankDrivetrain {
	private:
		pros::AbstractMotor& leftMotors;
		pros::AbstractMotor& rightMotors;
		QAngularVelocity maxMotorSpeed = 0.0;

		/**
		 * @brief The distance between the two drive sides
		 *
		 * @details Used for driveCurvature and other math things
		 *
		 */
		QLength trackWidth;

		/**
		 * @brief Max speed of the drivetrain
		 *
		 */
		QVelocity maxSpeed;
	public:

		TankDrivetrain(const QLength &trackWidth, const QVelocity &maxSpeed, pros::AbstractMotor &leftMotors,
		               pros::AbstractMotor &rightMotors, const QAngularVelocity &maxMotorSpeed);

		QVelocity getVelocity() {
			return ((mean(leftMotors.get_actual_velocity_all()) + mean(rightMotors.get_actual_velocity_all())) /
			        2.0) / maxMotorSpeed.Convert(revolution/minute) * this->getMaxSpeed();
		}

		void tankSteerVoltage(int32_t leftVoltage, int32_t rightVoltage) {
			this->leftMotors.move_voltage(leftVoltage);
			this->rightMotors.move_voltage(rightVoltage);

			Log(string_format("LeftVoltage: %d, RightVoltage: %d", leftVoltage, rightVoltage));
		}

		void reset() {
			Log("Reset Drivetrain Position");
			this->leftMotors.tare_position_all();
			this->rightMotors.tare_position_all();
		}

		QLength getDistanceSinceReset() {
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

		const QVelocity &getMaxSpeed() const;

		void setMaxSpeed(const QVelocity &maxSpeed);

		const QLength &getTrackWidth() const;

		void setTrackWidth(const QLength &trackWidth);

		~TankDrivetrain() = default;
	};

	TankDrivetrain::TankDrivetrain(const QLength &trackWidth, const QVelocity &maxSpeed, pros::AbstractMotor &leftMotors,
	                               pros::AbstractMotor &rightMotors, const QAngularVelocity &maxMotorSpeed)
			: trackWidth(trackWidth), maxSpeed(maxSpeed), leftMotors(leftMotors), rightMotors(rightMotors),
			  maxMotorSpeed(maxMotorSpeed) {}

	const QVelocity &TankDrivetrain::getMaxSpeed() const {
		return maxSpeed;
	}

	void TankDrivetrain::setMaxSpeed(const QVelocity &maxSpeed) {
		TankDrivetrain::maxSpeed = maxSpeed;
	}

	const QLength &TankDrivetrain::getTrackWidth() const {
		return trackWidth;
	}

	void TankDrivetrain::setTrackWidth(const QLength &trackWidth) {
		TankDrivetrain::trackWidth = trackWidth;
	}
} // namespace Pronounce




