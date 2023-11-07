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
		stateControllers.addBehavior(&catapultStateController);
		stateControllers.addBehavior(&wingsStateController);
		stateControllers.addBehavior(&teleopController);
		stateControllers.addBehavior(&loggerService);

		robotBehaviorMutex.give();
	}

	class ModeLogic : public Behavior {
	private:
		RobotStatus* robotStatus;
		uint32_t shotTriballs{0};
		QLength lastDistance{0.0};
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

			if (catapultDistance.get() * 1_mm < 1_in && lastDistance > 1_in) {
				shotTriballs += 1;
				catapultStateController.setCurrentBehavior(catapultFire.until([=]() -> bool {return catapultDistance.get() * 1_mm > 1_in;}));
			}
			lastDistance = catapultDistance.get() * 1_mm;

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

		void resetTriballs() {
			shotTriballs = 0;
		}

		uint32_t getTriballCount() {
			return shotTriballs;
		}

		~ModeLogic() = default;
	};
} // namespace Pronounce
