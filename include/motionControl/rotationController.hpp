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
		AbstractTankDrivetrain& drivetrain;
		ContinuousOdometry& odometry;
		Angle target;

		pros::motor_brake_mode_e_t beforeBrakeMode;

	public:
		RotationController(std::string name, AbstractTankDrivetrain& drivetrain, ContinuousOdometry& odometry, PID rotationPID, Angle target, pros::Mutex& drivetrainMutex) : drivetrain(drivetrain), rotationPID(rotationPID), odometry(odometry), Behavior(name), drivetrainMutex(drivetrainMutex) {
			rotationPID.setTarget(target.Convert(radian));
			this->target = target;
		}

		void initialize() {

			rotationPID.reset();
			rotationPID.setTarget(target.Convert(radian));

			beforeBrakeMode = leftDriveMotors.get_brake_modes().at(0);

			drivetrain.tankSteerVoltage(0, 0);
			leftDriveMotors.set_brake_modes(pros::E_MOTOR_BRAKE_COAST);
			rightDriveMotors.set_brake_modes(pros::E_MOTOR_BRAKE_COAST);
		}

		void update() {
			double output = rotationPID.update(odometry.getPose().getAngle().Convert(radian));

			std::cout << output << std::endl;
			std::cout << rotationPID.getError() << std::endl;

			drivetrain.tankSteerVoltage(output * 12000, -output * 12000);
		}

		void exit() {
			drivetrain.skidSteerVelocity(0.0, 0.0);
			leftDriveMotors.set_brake_modes(beforeBrakeMode);
			rightDriveMotors.set_brake_modes(beforeBrakeMode);

			leftDriveMotors.tare_position();
			rightDriveMotors.tare_position();

			drivetrain.tankSteerVoltage(0, 0);
		}

		bool isDone() {
			return false; // rotationPID.getError() < (1_deg).Convert(radian) && rotationPID.getDerivitive() < 0.00005;
		}

		~RotationController();
	};

	RotationController::~RotationController()
	{
	}
	
} // namespace Pronounce
