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
		TeleopModeLogic(pros::Controller* controller1, pros::Controller* controller2) {
			this->controller1 = controller1;
			this->controller2 = controller2;
		}

		void initialize() {
			drivetrainStateController.setCurrentBehavior(&normalJoystick);
		}

		void update() {
			if (controller1->get_digital_new_press(INTAKE_BUTTON)) {
				intakeStateController.setCurrentBehavior(intakeStateController.isDone() ? &intakeStopped : &intakeIntaking);
			}

			if (controller1->get_digital_new_press(LAUNCHER_STOP_BUTTON)) {
				launcherStateController.setCurrentBehavior(&launcherStopped);
			} else if (controller1->get_digital_new_press(PRIME_BUTTON)) {
				launcherStateController.setCurrentBehavior(&launcherIdle);
			} else if (controller1->get_digital(LAUNCH_BUTTON) && launcherStateExtensionController.isDone()) {
				launcherStateExtensionController.setCurrentBehavior(&launchDisc);
			}

			if (controller1->get_digital(DIGITAL_DOWN)) {
				flywheelAdjustment -= 2.0;
			}
			if (controller1->get_digital(DIGITAL_UP)) {
				flywheelAdjustment += 2.0;
			}

			if (controller1->get_digital(DIGITAL_LEFT)) {
				turretAngle -= 5.0;
			}
			if (controller1->get_digital(DIGITAL_RIGHT)) {
				turretAngle += 5.0;
			}

			if (controller2->is_connected()) {
				turretAngle += abs(controller2->get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X)) > 15 ? (double) controller2->get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X) / 50.0 : 0;
				flywheelAdjustment += abs(controller2->get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y)) > 15 ? (double) controller2->get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y) / 10.0 : 0;
			}

			std::cout << "Turret: " << turretAngle << std::endl;
			std::cout << "FlywheelSpeed: " << flywheelAdjustment << std::endl;
		}

		void exit() {

		}

		bool isDone() {
			return false;
		}

		~TeleopModeLogic() {}
	};
} // namespace Pronounce
