#pragma once

#include "behaviors/robotBehaviors.hpp"
#include "robotStatus.hpp"
#include "behavior.hpp"

namespace Pronounce {

	StateController stateExtensionController(new Behavior());

	BehaviorGroup stateControllers;

	void initBehaviors() {
		stateControllers.addBehavior(&intakeStateController);
		stateControllers.addBehavior(&launcherStateController);
		stateControllers.addBehavior(&launcherStateExtensionController);
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
		}

		void update() {
			robotStatus->update();
		}

		void exit() {
			robotStatus->exit();
		}

		bool isDone() {
			return false;
		}

		~ModeLogic() {}
	};
} // namespace Pronounce
