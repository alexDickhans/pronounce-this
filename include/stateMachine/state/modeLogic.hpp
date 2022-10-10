#pragma once

#include "stateMachine/behaviors/robotBehaviors.hpp"
#include "robotStatus.hpp"
#include "stateMachine/behavior.hpp"
#include "stateMachine/behaviorGroup.hpp"
#include "stateMachine/parallel.hpp"
#include "utils/utils.hpp"
#include "loggerService.hpp"

// TODO: Add docstrings

namespace Pronounce {

	LoggerService loggerService;

	StateController stateExtensionController("GlobalStateExtensionsController", new Behavior());

	BehaviorGroup stateControllers;

	StateController teleopController("TeleopController", new Behavior());

	void initBehaviors() {
		robotBehaviorMutex.take();

		stateControllers.addBehavior(&stateExtensionController);
		stateControllers.addBehavior(&ptoStateController);
		stateControllers.addBehavior(&drivetrainStateController);
		stateControllers.addBehavior(&endgameStateController);
		stateControllers.addBehavior(&teleopController);
		stateControllers.addBehavior(&loggerService);

		robotBehaviorMutex.give();
	}

	class ModeLogic : public Behavior {
	private:
		RobotStatus* robotStatus;
	public:
		ModeLogic(RobotStatus* robotStatus) {
			this->robotStatus = robotStatus;
		}

		void initialize() {
			robotBehaviorMutex.take();

			robotStatus->initialize();
			stateControllers.initialize();

			robotBehaviorMutex.give();
		}

		void update() {
			robotBehaviorMutex.take();

			robotStatus->update();
			stateControllers.update();

			robotBehaviorMutex.give();
		}

		void exit() {
			robotBehaviorMutex.take();

			robotStatus->exit();
			stateControllers.exit();

			robotBehaviorMutex.give();
		}

		bool isDone() {
			return false;
		}

		~ModeLogic() {}
	};
} // namespace Pronounce
