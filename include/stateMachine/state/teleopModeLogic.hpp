#pragma once

#include "stateMachine/behavior.hpp"
#include "stateMachine/behaviors/robotBehaviors.hpp"
#include "api.h"
#include "robotStatus.hpp"
#include "driver.h"
#include "modeLogic.hpp"

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
			drivetrainStateController.setCurrentBehavior(&fieldRelativeJoystick);
		}

		void update() {
			if (controller1->get_digital(DEJAM_BUTTON)) {
				intakeStateExtensionController.setCurrentBehavior(&intakeDejamSequence);
			} else if (controller1->get_digital_new_press(INTAKE_BUTTON)) {
				lastChangeFrame = pros::millis();
				intakeStateController.setCurrentBehavior(intakeStateController.isDone() ? &intakeStopped : &intakeIntaking);
			} else if (controller1->get_digital_new_press(ROLLER_BUTTON)) {
				lastChangeFrame = pros::millis();
				intakeStateController.setCurrentBehavior(intakeStateController.getCurrentBehavior() == &intakeEjecting ? &intakeStopped : &intakeEjecting);
			}

			if (controller1->get_digital_new_press(LAUNCHER_STOP_BUTTON)) {
				launcherStateController.setCurrentBehavior(&launcherStopped);
			} else if (controller1->get_digital_new_press(PRIME_BUTTON)) {
				launcherStateController.setCurrentBehavior(&launcherIdle);
			} else if (controller1->get_digital(LAUNCH_BUTTON) && launcherStateController.isDone()) {
				launcherStateExtensionController.setCurrentBehavior(&launchDisc);
			}

			if (controller1->get_digital(DIGITAL_DOWN)) {
				flywheelAdjustment -= 2.0;
			}
			if (controller1->get_digital(DIGITAL_UP)) {
				flywheelAdjustment += 2.0;
			}

			if (controller1->get_digital(DIGITAL_LEFT)) {
				turretAdjustment -= 0.005;
			}
			if (controller1->get_digital(DIGITAL_RIGHT)) {
				turretAdjustment += 0.005;
			}

			if (controller2->is_connected()) {
				turretAdjustment += abs(controller2->get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X)) > 15 ? (double) controller2->get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X) / 3000.0 : 0;
				flywheelAdjustment += abs(controller2->get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y)) > 15 ? (double) controller2->get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y) / 20.0 : 0;

				if (controller2->get_digital_new_press(DIGITAL_L2)) {
					robotStatus->setFlywheelRPM(2300);
					flywheelAdjustment = 0.0;
				}
				if (controller2->get_digital_new_press(DIGITAL_L1))
					robotStatus->setFlywheelRPM(2500);

				if (controller2->get_digital(DIGITAL_X) && controller1->get_digital(DIGITAL_X))
					endgameStateController.setCurrentBehavior(&endgameEnabled);

				if (pros::millis() % 200 < 15) {
					controller2->clear();
				} else {
					controller2->set_text(0, 0, std::to_string(this->robotStatus->getFlywheelTarget()));
				}
			} else {
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
