#pragma once

#include "chassis/abstractHolonomicDrivetrain.hpp"
#include "feedbackControllers/pid.hpp"
#include "stateMachine/behavior.hpp"
#include "stateMachine/behaviors/drivetrain/initDrivetrain.hpp"

namespace Pronounce
{
	class RotationController : public Behavior {
	private:
		PID* rotationPID;
		AbstractHolonomicDrivetrain* drivetrain;
		ContinuousOdometry* odometry;

	public:
		RotationController(std::string name, AbstractHolonomicDrivetrain* drivetrain, ContinuousOdometry* odometry, PID* rotationPID, Angle target) : drivetrain(drivetrain), rotationPID(rotationPID), odometry(odometry), Behavior(name) {
			rotationPID->setTurnPid(true);
			rotationPID->setTarget(target.Convert(radian));
		}

		void initialize() {
			rotationPID->reset();
		}

		void update() {
			double output = rotationPID->update(odometry->getPose()->getAngle().Convert(radian));

			drivetrain->setDriveVectorVelocity(Vector(), output);
		}

		void exit() {
			drivetrain->setDriveVectorVelocity(Vector(), 0.0);
		}

		bool isDone() {
			return rotationPID->getError() < (0.05_rad).Convert(radian) && rotationPID->getDerivitive() < 0.0005;
		}

		~RotationController();
	};

	RotationController::~RotationController()
	{
	}
	
} // namespace Pronounce
