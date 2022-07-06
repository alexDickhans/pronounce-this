#pragma once

#include "stateMachine/behaviors/robotBehaviors.hpp"
#include "robotStatus.hpp"
#include "stateMachine/behavior.hpp"
#include "stateMachine/behaviorGroup.hpp"
#include "utils/utils.hpp"

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
			launcherIdle.setFlywheelSpeed(robotStatus->getFlywheelTarget());
			launcherLaunching.setFlywheelSpeed(robotStatus->getFlywheelTarget());
			launcherFullSpeed.setFlywheelSpeed(robotStatus->getFlywheelTarget());

			launcherStopped.setTurretAngle(clamp(robotStatus->getTurretAngle() - toDegrees(angleDifference(odometry.getPosition()->getTheta(), 0)), 0.0, 180.0));
			launcherIdle.setTurretAngle(clamp(robotStatus->getTurretAngle() - toDegrees(angleDifference(odometry.getPosition()->getTheta(), 0.0)), 0.0, 180.0));
			launcherLaunching.setTurretAngle(clamp(robotStatus->getTurretAngle() - toDegrees(angleDifference(odometry.getPosition()->getTheta(), 0.0)), 0.0, 180.0));
			launcherFullSpeed.setTurretAngle(clamp(robotStatus->getTurretAngle() - toDegrees(angleDifference(odometry.getPosition()->getTheta(), 0.0)), 0.0, 180.0));

			std::cout << "Turret angle: " << robotStatus->getTurretAngle() - toDegrees(angleDifference(odometry.getPosition()->getTheta(), 0.0)) << std::endl;
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
