#pragma once

#include "chassis/abstractTankDrivetrain.hpp"
#include "feedbackControllers/pid.hpp"
#include "stateMachine/behavior.hpp"
#include "stateMachine/behaviors/drivetrain/initDrivetrain.hpp"
#include "pros/rtos.hpp"
#include "hardware/hardware.hpp"

namespace Pronounce
{
	class RotationController : public Behavior {
	private:
		pros::Mutex& drivetrainMutex;
		PID rotationPID;
		TankDrivetrain& drivetrain;
		ContinuousOdometry& odometry;
		Angle target;

		pros::MotorBrake beforeBrakeMode;

		double idleSpeed = 0.0;

	public:
		RotationController(std::string name, TankDrivetrain& drivetrain, ContinuousOdometry& odometry, PID rotationPID, Angle target, pros::Mutex& drivetrainMutex, double idleSpeed = 0.0) : drivetrain(drivetrain), rotationPID(rotationPID), odometry(odometry), Behavior(name), drivetrainMutex(drivetrainMutex) {
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
			double output = rotationPID.update(odometry.getPose().getAngle().Convert(radian));

			drivetrain.tankSteerVoltage(output * 12000 + idleSpeed, -output * 12000 + idleSpeed);
		}

		void exit() {
			drivetrain.setBrakeMode(beforeBrakeMode);

			leftDriveMotors.tare_position();
			rightDriveMotors.tare_position();

			drivetrain.tankSteerVoltage(0, 0);
		}

		bool isDone() {
			return false; // rotationPID.getError() < (1_deg).Convert(radian) && rotationPID.getDerivitive() < 0.00005;
		}

		~RotationController() = default;
	};
	
} // namespace Pronounce
