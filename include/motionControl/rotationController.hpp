#pragma once

#include <utility>

#include "chassis/tankDrive.hpp"
#include "feedbackControllers/pid.hpp"
#include "stateMachine/behavior.hpp"
#include "stateMachine/behaviors/drivetrain/initDrivetrain.hpp"
#include "pros/rtos.hpp"
#include "hardware/hardware.hpp"

namespace Pronounce
{
	class RotationController : public Behavior {
	private:
		PID rotationPID;
		TankDrivetrain& drivetrain;
		std::function<Angle()> angleFunction;
		Angle target;

		pros::MotorBrake beforeBrakeMode;

		double idleSpeed = 0.0;

	public:
		RotationController(std::string name, TankDrivetrain& drivetrain, std::function<Angle()> angleFunction, PID rotationPID, Angle target, double idleSpeed = 0.0) : drivetrain(drivetrain), rotationPID(rotationPID), angleFunction(std::move(angleFunction)), Behavior(name) {
			rotationPID.setTarget(target.Convert(radian));
			this->idleSpeed = idleSpeed;
			this->target = target;
		}

		void initialize() {

			rotationPID.reset();
			rotationPID.setTarget(target.Convert(radian));

			beforeBrakeMode = drivetrain.getBrakeMode();

			drivetrain.tankSteerVoltage(0, 0);
			drivetrain.setBrakeMode(pros::MotorBrake::coast);
		}

		void update() {
			double output = rotationPID.update(angleFunction().Convert(radian));

			drivetrain.tankSteerVoltage(output * 12000 + idleSpeed, -output * 12000 + idleSpeed);
		}

		void exit() {
			drivetrain.setBrakeMode(beforeBrakeMode);

			leftDriveMotors.tare_position();
			rightDriveMotors.tare_position();

			drivetrain.tankSteerVoltage(0, 0);
		}

		bool isDone() {
			return false;
		}

		~RotationController() = default;
	};
	
} // namespace Pronounce
