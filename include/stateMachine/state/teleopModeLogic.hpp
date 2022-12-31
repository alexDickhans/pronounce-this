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
			leftPtoMotor.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
			rightPtoMotor.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
		}

		void update() {

			if (controller1->get_digital_new_press(LAUNCH_BUTTON)) {
				drivetrainStateController.setCurrentBehavior(&normalJoystick);
				ptoStateExtensionController.setCurrentBehavior(&ptoCatapultLaunch);
			}
			std::cout << "IntakeStatus: " << ptoStateController.isDone() << std::endl;

			if (controller1->get_digital_new_press(DIGITAL_L1) && ptoStateController.getCurrentBehavior() != &ptoCatapult) {
				ptoStateController.setCurrentBehavior(ptoStateController.isDone() ? &ptoIntakeStopped : &ptoIntaking);
			}

			if (controller1->get_digital_new_press(DIGITAL_L2)) {
				drivetrainStateController.setCurrentBehavior(drivetrainStateController.getCurrentBehavior() == &targetingJoystick ? &normalJoystick : &targetingJoystick);
			}

			if (controller2->get_digital(DIGITAL_L2) && controller2->get_digital(DIGITAL_R2))
				endgameStateController.setCurrentBehavior(&endgameEnabled);
			
			if (controller2->get_digital(pros::E_CONTROLLER_DIGITAL_DOWN)) {
				pistonBoostStateController.setCurrentBehavior(&pistonBoostOverfill);
			}

			if (controller2->get_digital(pros::E_CONTROLLER_DIGITAL_LEFT)) {
				pistonBoostStateController.setCurrentBehavior(&pistonBoostNone);
			}

			if (controller2->get_digital(pros::E_CONTROLLER_DIGITAL_UP)) {
				pistonBoostStateController.setCurrentBehavior(&pistonBoostBoosting);
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
