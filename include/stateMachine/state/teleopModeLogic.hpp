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

			controller2->clear();
			controller1->clear();
		}

		void update() {

			if (controller1->get_digital_new_press(LAUNCH_BUTTON)) {
				if (autoVisionAim) {
					drivetrainStateController.setCurrentBehavior(&targetingJoystickStop);
				} else {
					drivetrainStateController.setCurrentBehavior(&normalJoystick);
				}
				ptoStateExtensionController.setCurrentBehavior(&ptoCatapultLaunch);
			}

			

			std::cout << "IntakeStatus: " << ptoStateController.isDone() << std::endl;

			if (controller1->get_digital_new_press(DIGITAL_L1)) {
				ptoStateController.setCurrentBehavior(ptoStateController.isDone() ? &ptoIntakeStopped : &ptoIntaking);
			}

			if (controller1->get_digital_new_press(DIGITAL_L2)) {
				drivetrainStateController.setCurrentBehavior(drivetrainStateController.getCurrentBehavior() == &targetingJoystick ? &normalJoystick : &targetingJoystick);
			}
			
			if (controller2->get_digital_new_press(DIGITAL_L2)) {
				ptoStateExtensionController.useDefaultBehavior();
				ptoStateController.setCurrentBehavior(&ptoIntakeStopped);
			}

			if (controller2->get_digital_new_press(DIGITAL_L1)) {
				ptoStateExtensionController.useDefaultBehavior();
				ptoStateController.setCurrentBehavior(&ptoIntaking);
			}

			if (controller2->get_digital_new_press(DIGITAL_R1)) {
				ptoStateExtensionController.setCurrentBehavior(&ptoCatapultLaunch);
			}
			
			if (controller2->get_digital_new_press(DIGITAL_R2)) {
				hardwareOverride = !hardwareOverride;
				controller2->clear_line(2);
			}
			
			controller2->set_text(2, 0, (std::to_string(hardwareOverride) + "     " + std::to_string(autoVisionAim)).c_str());

			if (controller2->get_digital_new_press(DIGITAL_X)) {
				autoVisionAim = !autoVisionAim;
				controller2->clear_line(1);
			}

			if (controller2->get_digital(DIGITAL_DOWN) && controller2->get_digital(DIGITAL_B))
				endgameStateController.setCurrentBehavior(&endgameEnabled);

			if (controller2->get_digital(pros::E_CONTROLLER_DIGITAL_DOWN)) {
				pistonBoostStateController.setCurrentBehavior(&pistonBoostOverfill);
			}

			if (controller2->get_digital(pros::E_CONTROLLER_DIGITAL_LEFT)) {
				pistonBoostStateController.setCurrentBehavior(&pistonBoostNone);
			}

			if (controller2->get_digital(pros::E_CONTROLLER_DIGITAL_RIGHT)) {
				pistonBoostStateController.setCurrentBehavior(&pistonBoostBoth);
			}

			if (controller2->get_digital(pros::E_CONTROLLER_DIGITAL_UP)) {
				pistonBoostStateController.setCurrentBehavior(&pistonBoostBoosting);
			}

			if (controller2->get_digital_new_press(pros::E_CONTROLLER_DIGITAL_A) && gameMode != GameMode::Skills) {
				gameMode = gameMode == GameMode::Red ? GameMode::Blue : GameMode::Red;
			}

			intakeSolenoid.set_value(controller2->get_digital(pros::E_CONTROLLER_DIGITAL_A));
		}

		void exit() {

		}

		bool isDone() {
			return false;
		}

		~TeleopModeLogic() {}
	};
} // namespace Pronounce
