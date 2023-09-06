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

	BehaviorGroup stateControllers;

	StateController teleopController("TeleopController", new Behavior());

	void initBehaviors() {

		robotBehaviorMutex.take();

		stateControllers.addBehavior(&drivetrainStateController);
		stateControllers.addBehavior(&intakeExtensionStateController);
		stateControllers.addBehavior(&intakeStateController);
		stateControllers.addBehavior(&blockerStateController);
		stateControllers.addBehavior(&teleopController);
		stateControllers.addBehavior(&loggerService);

		robotBehaviorMutex.give();
	}

	class ModeLogic : public Behavior {
	private:
		RobotStatus* robotStatus;
	public:
		explicit ModeLogic(RobotStatus* robotStatus) {
			this->robotStatus = robotStatus;
		}

		void initialize() override {
			robotBehaviorMutex.take();

			robotStatus->initialize();
			stateControllers.initialize();

			robotBehaviorMutex.give();
		}

		void update() override {
			robotBehaviorMutex.take();

			robotStatus->update();
			stateControllers.update();

			robotBehaviorMutex.give();
		}

		void exit() override {
			robotBehaviorMutex.take();

			robotStatus->exit();
			stateControllers.exit();

			robotBehaviorMutex.give();
		}

		bool isDone() override {
			return false;
		}

		~ModeLogic() = default;
	};
} // namespace Pronounce
