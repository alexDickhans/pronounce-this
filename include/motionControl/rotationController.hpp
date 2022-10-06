#pragma once

#include "chassis/abstractTankDrivetrain.hpp"
#include "feedbackControllers/pid.hpp"
#include "stateMachine/behavior.hpp"
#include "stateMachine/behaviors/drivetrain/initDrivetrain.hpp"
#include "pros/rtos.hpp"

namespace Pronounce
{
	class RotationController : public Behavior {
	private:
		pros::Mutex& drivetrainMutex;
		PID& rotationPID;
		AbstractTankDrivetrain& drivetrain;
		ContinuousOdometry& odometry;

	public:
		RotationController(std::string name, AbstractTankDrivetrain& drivetrain, ContinuousOdometry& odometry, PID& rotationPID, Angle target, pros::Mutex drivetrainMutex) : drivetrain(drivetrain), rotationPID(rotationPID), odometry(odometry), Behavior(name), drivetrainMutex(drivetrainMutex) {
			rotationPID.setTurnPid(true);
			rotationPID.setTarget(target.Convert(radian));
		}

		void initialize() {
			rotationPID.reset();
		}

		void update() {
			double output = rotationPID.update(odometry.getPose()->getAngle().Convert(radian));

			drivetrain.skidSteerVelocity(0.0, output);
		}

		void exit() {
			drivetrain.skidSteerVelocity(0.0, 0.0);
		}

		bool isDone() {
			return rotationPID.getError() < (0.05_rad).Convert(radian) && rotationPID.getDerivitive() < 0.0005;
		}

		~RotationController();
	};

	RotationController::~RotationController()
	{
	}
	
} // namespace Pronounce
