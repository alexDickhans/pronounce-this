#pragma once

#include "stateMachine/behavior.hpp"
#include "stateMachine/behaviors/robotBehaviors.hpp"
#include "api.h"
#include "robotStatus.hpp"
#include "driver.h"
#include "modeLogic.hpp"
#include "hardware/hardware.hpp"
#include "hardwareAbstractions/joystick/joystick.hpp"

// TODO: Clean up
// TODO: Add docstring

namespace Pronounce {
	class TeleopModeLogic : public Behavior {
	private:
		RobotStatus* robotStatus{};
		bool blockerStatus{true};

        AbstractJoystick* controller1;
        AbstractJoystick* controller2;
	public:
		TeleopModeLogic(AbstractJoystick* controller1, AbstractJoystick* controller2) {
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

			controller1->onPressed(E_CONTROLLER_DIGITAL_UP, [&] () -> void {
				blockerStatus = !blockerStatus;
				if (blockerStateController.getCurrentBehavior() != nullptr) {
					blockerStateController.setCurrentBehavior((blockerStatus ? blockerHigh : blockerMatchLoad).until([=] () -> bool {
						return controller1->get_digital_new_press(E_CONTROLLER_DIGITAL_L2);}));
				}
			});

			controller1->onPressed(E_CONTROLLER_DIGITAL_L2, [&] () -> void {
				blockerStateController.setCurrentBehavior((blockerStatus ? blockerHigh : blockerMatchLoad).until([=] () -> bool {
					return controller1->get_digital_new_press(E_CONTROLLER_DIGITAL_L2);}));
			});

			controller1->onPressed(E_CONTROLLER_DIGITAL_R1, [=] () -> void {
				intakeExtensionStateController.setCurrentBehavior(&intakeSequence);
			});

            controller1->onPressed(E_CONTROLLER_DIGITAL_R2, [=] () -> void {
				intakeExtensionStateController.useDefaultBehavior();
                intakeStateController.setCurrentBehavior(intakeEject.until([=] () -> bool {
                    return !controller1->get_digital(E_CONTROLLER_DIGITAL_R2);}));
			});

			controller1->onPressed(E_CONTROLLER_DIGITAL_L1, [&] () -> void {
				wingsStateController.setCurrentBehavior(wingsOut.until([&] () -> bool {
					return controller1->get_digital_new_press(E_CONTROLLER_DIGITAL_L1);
				}));
			});
		}

		void update() override {
			controller1->update();
			controller2->update();
		}

		void exit() override {
			drivetrainStateController.setDefaultBehavior(&drivetrainStopped);
			drivetrainStateController.useDefaultBehavior();
			controller1->clearCallbacks();
			controller2->clearCallbacks();
		}

		bool isDone() override {
			return false;
		}

		~TeleopModeLogic() = default;
	};
} // namespace Pronounce
