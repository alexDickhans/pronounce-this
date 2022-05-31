#pragma once

#include "stateMachine/behaviors/robotBehaviors.hpp"
#include "robotStatus.hpp"
#include "stateMachine/behavior.hpp"
#include "stateMachine/behaviorGroup.hpp"

namespace Pronounce {

	StateController stateExtensionController(new Behavior());

	BehaviorGroup stateControllers;

	StateController teleopController(new Behavior());

	void initBehaviors() {
		stateControllers.addBehavior(&teleopController);
		stateControllers.addBehavior(&stateExtensionController);
		stateControllers.addBehavior(&intakeStateController);
		stateControllers.addBehavior(&launcherStateExtensionController);
		stateControllers.addBehavior(&launcherStateController);
		stateControllers.addBehavior(&drivetrainStateController);
	}

	void initSequences() {
		
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
			initBehaviors();
			stateControllers.initialize();
			initSequences();
		}

		void update() {
			robotStatus->update();
			launcherIdle.setFlywheelSpeed(robotStatus->getFlywheelRpm());
			launcherLaunching.setFlywheelSpeed(robotStatus->getFlywheelRpm());
			launcherFullSpeed.setFlywheelSpeed(robotStatus->getFlywheelRpm());

			launcherIdle.setTurretAngle(robotStatus->getTurretAngle());
			launcherLaunching.setTurretAngle(robotStatus->getTurretAngle());
			launcherFullSpeed.setTurretAngle(robotStatus->getTurretAngle());
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
