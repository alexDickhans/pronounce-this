#pragma once

#include "stateMachine/behavior.hpp"
#include "stateMachine/behaviors/robotBehaviors.hpp"
#include "api.h"
#include "robotStatus.hpp"
#include "driver.h"
#include "modeLogic.hpp"
#include "hardware/hardware.hpp"

// TODO: Clean up
// TODO: Add docstring

namespace Pronounce {
	class TeleopModeLogic : public Behavior {
	private:
		RobotStatus* robotStatus;

		pros::Controller* controller1;
		pros::Controller* controller2;

		uint32_t lastChangeFrame = 0;

		bool intaking = true;
	public:
		TeleopModeLogic(pros::Controller* controller1, pros::Controller* controller2) {
			this->controller1 = controller1;
			this->controller2 = controller2;
		}

		void initialize() {
			drivetrainStateController.setCurrentBehavior(&normalJoystick);

			leftDriveMotors.set_brake_modes(pros::E_MOTOR_BRAKE_COAST);
			rightDriveMotors.set_brake_modes(pros::E_MOTOR_BRAKE_COAST);
			intakeMotors.set_brake_modes(pros::E_MOTOR_BRAKE_COAST);

			controller2->clear();
			controller1->clear();
		}

		void update() {
			if (controller1->get_digital_new_press(DIGITAL_R1)) {
				intakeStateController.setCurrentBehavior(&intakeIntaking);
			}
			if (!controller1->get_digital(DIGITAL_R1) && intakeStateController.getCurrentBehavior() == &intakeIntaking) {
				intakeStateController.setCurrentBehavior(hasTriball ? &intakeHold : &intakeStopped);
			}


			if (controller1->get_digital_new_press(DIGITAL_R2)) {
				intakeStateController.setCurrentBehavior(&intakeEject);
			}
			if (!controller1->get_digital(DIGITAL_R2) && intakeStateController.getCurrentBehavior() == &intakeEject) {
				intakeStateController.setCurrentBehavior(&intakeStopped);
			}

		}

		void exit() {
		}

		bool isDone() {
			return false;
		}

		~TeleopModeLogic() {}
	};
} // namespace Pronounce
