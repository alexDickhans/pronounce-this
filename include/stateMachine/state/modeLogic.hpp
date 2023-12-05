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
		stateControllers.addBehavior(&leftWingStateController);
		stateControllers.addBehavior(&rightWingStateController);
		stateControllers.addBehavior(&blockerStateController);
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

			// See if the distance sensor detects a new object within 1 inch of the sensor
			if (catapultDistance.get() * 1_mm < 0.75_in // see if an object is detected by the distance sensor on the catapult
			&& lastDistance > 0.75_in && blockerStateController.getCurrentBehavior() != &blockerIn) { // If the last distance sensor reading was greater than an inch indicates that the
				                      // triball is moving closer to the sensor, meaning that there is a new triball

				// increase the count of shot triballs
				shotTriballs += 1;

				// Set the catapult to try to shoot the triball until it has left the catapult
				catapultStateController.setCurrentBehavior(catapultFire.until([=]() -> bool {return catapultDistance.get() * 1_mm > 0.75_in;}));
			}

			// Store the last distance for the next loop itteration
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
