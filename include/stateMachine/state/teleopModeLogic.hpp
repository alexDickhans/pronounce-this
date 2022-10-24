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

		bool tilter;

		uint32_t lastChangeFrame = 0;
	public:
		TeleopModeLogic(pros::Controller* controller1, pros::Controller* controller2) {
			this->controller1 = controller1;
			this->controller2 = controller2;
		}

		void initialize() {
			drivetrainStateController.setCurrentBehavior(&normalJoystick);
			
			leftDriveMotors.set_brake_modes(pros::E_MOTOR_BRAKE_COAST);
			rightDriveMotors.set_brake_modes(pros::E_MOTOR_BRAKE_COAST);
			leftPtoMotor.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
			rightPtoMotor.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
		}

		void update() {

			if (controller1->get_digital_new_press(LAUNCH_BUTTON)) {
				drivetrainStateController.setCurrentBehavior(&normalJoystick);
				ptoStateExtensionController.setCurrentBehavior(&ptoCatapultLaunch);
			}

			if (controller1->get_digital_new_press(INTAKE_BUTTON)) {
				ptoStateController.setCurrentBehavior(&ptoIntaking);
			}

			if (controller1->get_digital_new_press(PTO_DRIVE_BUTTON)) {
				ptoStateController.setCurrentBehavior(&ptoDrive);
			}

			if (controller1->get_digital_new_press(DIGITAL_L2)) {
				drivetrainStateController.setCurrentBehavior(drivetrainStateController.getCurrentBehavior() == &targetingJoystick ? &normalJoystick : &targetingJoystick);
			}

			if (controller2->is_connected()) {
				if (controller2->get_digital(DIGITAL_X) && controller1->get_digital(DIGITAL_X))
					endgameStateController.setCurrentBehavior(&endgameEnabled);
			}
			else {
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
