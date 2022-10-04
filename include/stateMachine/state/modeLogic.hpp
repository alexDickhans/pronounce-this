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
		stateControllers.addBehavior(&ptoStateController);
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
