#pragma once

#include "stateMachine/behaviors/robotBehaviors.hpp"
#include "robotStatus.hpp"
#include "stateMachine/behavior.hpp"
#include "stateMachine/behaviorGroup.hpp"
#include "stateMachine/parallel.hpp"
#include "utils/utils.hpp"

// TODO: clean up
// TODO: Add docstrings

namespace Pronounce {

	StateController stateExtensionController("GlobalStateExtensionsController", new Behavior());

	BehaviorGroup stateControllers;

	StateController teleopController("TeleopController", new Behavior());

	void initBehaviors() {
		stateControllers.addBehavior(&stateExtensionController);
		stateControllers.addBehavior(&intakeStateController);
		stateControllers.addBehavior(&launcherStateExtensionController);
		stateControllers.addBehavior(&launcherStateController);
		stateControllers.addBehavior(&drivetrainStateController);
		stateControllers.addBehavior(&endgameStateController);
		stateControllers.addBehavior(&teleopController);
	}

	class ModeLogic : public Behavior {
	private:
		RobotStatus* robotStatus;
	public:
		ModeLogic(RobotStatus* robotStatus) {
			this->robotStatus = robotStatus;
		}

		void initialize() {
			robotStatus->initialize();
			stateControllers.initialize();
		}

		void update() {
			robotStatus->update();
			launcherIdle.setFlywheelSpeed(robotStatus->getFlywheelTarget());
			launcherLaunching.setFlywheelSpeed(robotStatus->getFlywheelTarget());
			launcherFullSpeed.setFlywheelSpeed(robotStatus->getFlywheelTarget());

			double turretAngle = 0;
			
			if (false) {
				turretAngle = clamp(angleDifference(robotStatus->getTurretAngle().Convert(radian), 0), -M_PI_2, M_PI_2);
			} else {
				turretAngle = clamp(angleDifference((robotStatus->getTurretAngle() - odometry.getAngle()).getValue(), 0), -M_PI_2, M_PI_2);
			}

			launcherStopped.setTurretAngle(turretAngle);
			launcherIdle.setTurretAngle(turretAngle);
			launcherLaunching.setTurretAngle(turretAngle);
			launcherFullSpeed.setTurretAngle(turretAngle);

			// Intake dejam

			std::cout << "InputTurretAngle: " << turretAngle << std::endl;
			stateControllers.update();
		}

		void exit() {
			robotStatus->exit();
			stateControllers.exit();
		}

		bool isDone() {
			return false;
		}

		~ModeLogic() {}
	};
} // namespace Pronounce
