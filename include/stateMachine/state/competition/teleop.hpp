#pragma once

#include "stateMachine/behavior.hpp"
#include "stateMachine/behaviors/robotBehaviors.hpp"
#include "api.h"
#include "hardware/hardware.hpp"
#include "hardwareAbstractions/joystick/joystick.hpp"
#include "enabled.hpp"

namespace Pronounce {

	class Teleop : public Enabled {
	private:
		AbstractJoystick* controller1;
		AbstractJoystick* controller2;
		pros::Controller controller1pros;
		int hangIndex = 6;
	public:
		Teleop(AbstractJoystick* controller1, AbstractJoystick* controller2) : Enabled("Teleop"), controller1pros(pros::E_CONTROLLER_MASTER) {
			this->controller1 = controller1;
			this->controller2 = controller2;
		}

		void initialize() override {
			drivetrainStateController.setDefaultBehavior(&normalJoystick);
			drivetrainStateController.useDefaultBehavior();

			leftDriveMotors.set_brake_modes(pros::E_MOTOR_BRAKE_COAST);
			rightDriveMotors.set_brake_modes(pros::E_MOTOR_BRAKE_COAST);
			intakeMotors.set_brake_modes(pros::E_MOTOR_BRAKE_COAST);

			controller1->clearCallbacks();
			controller2->clearCallbacks();

			controller1->onPressed(E_CONTROLLER_DIGITAL_L2, [&] () -> void {

				if (!hangReleaseStateController.isDone()) {
					drivetrainStateController(&hang);
				} else {
					leftWingStateController.setCurrentBehavior(leftWingOut.until([=] () -> bool {
						return !controller1->get_digital(E_CONTROLLER_DIGITAL_L2);
					}));
					rightWingStateController.setCurrentBehavior(rightWingOut.until([&] () -> bool {
						return !controller1->get_digital(E_CONTROLLER_DIGITAL_L2);
					}));
				}
			});

			controller1->onPressed(E_CONTROLLER_DIGITAL_R2, [&] () -> void {
				intakeExtensionStateController.useDefaultBehavior();
				intakeStateController.setCurrentBehavior(intakeEject.until([=] () -> bool {
					return !controller1->get_digital(E_CONTROLLER_DIGITAL_R2);}));

				if (!hangReleaseStateController.isDone()) {
					hangReleaseStateController(&hangReleaseDown);
				}
			});

			controller1->onPressed(E_CONTROLLER_DIGITAL_R1, [=] () -> void {
				intakeExtensionStateController.setCurrentBehavior(&intakeSequence);
			});

			controller1->onPressed(E_CONTROLLER_DIGITAL_L1, [=] () -> void {
				awpStateController.setCurrentBehavior(awpOut.until([&] () -> bool {
					return !controller1->get_digital(E_CONTROLLER_DIGITAL_L1);
				}));
			});

			controller1->onPressed(E_CONTROLLER_DIGITAL_DOWN, [&] () -> void {
				hangIndex--;
			});

			controller1->onPressed(E_CONTROLLER_DIGITAL_UP, [&] () -> void {
				hangIndex++;

				hang.setTargetPosition(hangMap[hangList[hangIndex]]);

				if (hang.hasHung()) {
					drivetrainStateController(&hang);
				}
			});

			Enabled::initialize();
		}

		void update() override {
			Enabled::update();

			controller1->update();
			controller2->update();

			if (controller1->get_digital(E_CONTROLLER_DIGITAL_A) && controller1->get_digital(E_CONTROLLER_DIGITAL_LEFT) && drivetrainStateController.isDone()) {
				hangReleaseStateController(&hangReleaseOut);
			}

			hang.setTier(hangList[hangIndex % 7]);

			if (pros::millis() % 100 / 10 == 0 || pros::millis() % 100 / 10 == 5) {

				char* upperTier = (char*) calloc((2+hangIndex), sizeof(char));

				snprintf(upperTier, 2, "%c", toupper(hangList[hangIndex % 7]));

				controller1pros.set_text(1, 1, upperTier);

				free(upperTier);
			}
		}

		void exit() override {
			drivetrainStateController.setDefaultBehavior(&drivetrainStopped);
			drivetrainStateController.useDefaultBehavior();
			controller1->clearCallbacks();
			controller2->clearCallbacks();
			Enabled::exit();
		}

		bool isDone() override {
			return false;
		}

		~Teleop() = default;
	};
} // namespace Pronounce