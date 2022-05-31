#pragma once

#include "stateMachine/behavior.hpp"
#include "stateMachine/behaviors/robotBehaviors.hpp"
#include "api.h"
#include "robotStatus.hpp"
#include "driver.h"

namespace Pronounce {
	class TeleopModeLogic : public Behavior {
	private:
		RobotStatus* robotStatus;

		pros::Controller* controller1;
		pros::Controller* controller2;
	public:
		TeleopModeLogic() {}

		void initialize() {

		}

		void update() {
			if (controller1->get_digital(INTAKE_BUTTON)) {
				intakeStateController.setCurrentBehavior(intakeStateController.isDone() ? intakeStopped : intakeIntaking);
			}
		}

		void exit() {

		}

		bool isDone() {

		}

		~TeleopModeLogic() {}
	};
} // namespace Pronounce
