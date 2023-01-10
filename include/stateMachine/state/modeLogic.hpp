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

	Sequence rollerSequence("RollerSequence");
	Sequence rollerSequence2("RollerSequence");
	Sequence skillsRollerSequence("SkillsRollerSequence");

	void initBehaviors() {

		robotBehaviorMutex.take();

		rollerSequence.addState(&drivetrainStateController, &moveForward5in);
		rollerSequence.addState(&drivetrainStateController, &moveBackward5in);
		rollerSequence.addState(&drivetrainStateController, &moveForward5in);
		rollerSequence.addState(&drivetrainStateController, &moveBackward5in);
		rollerSequence.addState(&drivetrainStateController, &moveBackward5in);

		rollerSequence2.addState(&drivetrainStateController, &moveForward5in);
		rollerSequence2.addState(&drivetrainStateController, &moveBackward5in);
		rollerSequence2.addState(&drivetrainStateController, &moveForward5in);

		skillsRollerSequence.addState(&drivetrainStateController, &moveForward5in);
		skillsRollerSequence.addState(&drivetrainStateController, &moveBackward5in);

		stateControllers.addBehavior(&stateExtensionController);
		stateControllers.addBehavior(&ptoStateController);
		stateControllers.addBehavior(&ptoStateExtensionController);
		stateControllers.addBehavior(&drivetrainStateController);
		stateControllers.addBehavior(&pistonBoostStateController);
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

			if ((ptoStateController.isDone() || ptoStateController.getCurrentBehavior() == &ptoIntakeStopped) && (catapultLimitSwitch.get_angle() < 34500 && catapultLimitSwitch.get_angle() > 1500) && !hardwareOverride) {
				ptoStateController.setCurrentBehavior(&ptoCatapult);
			}

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
