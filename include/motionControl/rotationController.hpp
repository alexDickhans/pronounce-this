#pragma once

#include <utility>

#include "chassis/tankDrive.hpp"
#include "feedbackControllers/pid.hpp"
#include "stateMachine/behavior.hpp"
#include "stateMachine/behaviors/drivetrain/initDrivetrain.hpp"
#include "pros/rtos.hpp"
#include "hardware/hardware.hpp"

namespace Pronounce {
	enum RotationOptimizer {
		none,
		closest,
		clockwise,
		counterclockwise
	};

	class RotationController : public Behavior {
	private:
		PID rotationPID;
		TankDrivetrain& drivetrain;
		std::function<Angle()> angleFunction;
		Angle target;

		pros::MotorBrake beforeBrakeMode;

		double idleSpeed = 0.0;

		RotationOptimizer rotationOptimizer;

	public:
		RotationController(std::string name, TankDrivetrain& drivetrain, std::function<Angle()> angleFunction, PID rotationPID, Angle target, double idleSpeed = 0.0, RotationOptimizer rotationOptimizer = RotationOptimizer::none) : drivetrain(drivetrain), rotationPID(rotationPID), angleFunction(std::move(angleFunction)), Behavior(name), rotationOptimizer(rotationOptimizer) {
			rotationPID.setTarget(target.Convert(radian));
			this->idleSpeed = idleSpeed;
			this->target = target;
		}

		void initialize() {

			Angle currentPosition = angleFunction();

			switch (rotationOptimizer) {
				case closest:
					target = currentPosition + angleDifference(target.getValue(), currentPosition.getValue()) * 1_rad;
					break;
				case clockwise:
					target = angleDifference(target.getValue(), currentPosition.getValue()) * 1_rad;
					if (target.Convert(radian) < 0.0) {
						target += 360_deg;
					}
					target += currentPosition;
					break;
				case counterclockwise:
					target = angleDifference(target.getValue(), currentPosition.getValue()) * 1_rad;
					if (target.Convert(radian) > 0.0) {
						target -= 360_deg;
					}
					target += currentPosition;
					break;
				case none:
				default:
					break;
			}

			rotationPID.reset();
			rotationPID.setTarget(target.Convert(radian));

			beforeBrakeMode = drivetrain.getBrakeMode();

			drivetrain.tankSteerVoltage(0, 0);
			drivetrain.setBrakeMode(pros::MotorBrake::coast);
		}

		void update() {
			double output = rotationPID.update(angleFunction().Convert(radian));

			drivetrain.tankSteerVoltage(output + idleSpeed, -output + idleSpeed);
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
