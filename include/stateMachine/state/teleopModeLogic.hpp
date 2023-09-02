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

        AbstractJoystick* controller1;
        AbstractJoystick* controller2;

		uint32_t lastChangeFrame = 0;
	public:
		TeleopModeLogic(AbstractJoystick* controller1, AbstractJoystick* controller2) {
			this->controller1 = controller1;
			this->controller2 = controller2;
		}

		void initialize() {
			drivetrainStateController.setCurrentBehavior(&normalJoystick);

			leftDriveMotors.set_brake_modes(pros::E_MOTOR_BRAKE_COAST);
			rightDriveMotors.set_brake_modes(pros::E_MOTOR_BRAKE_COAST);
			intakeMotors.set_brake_modes(pros::E_MOTOR_BRAKE_COAST);

            controller1->onPressed(E_CONTROLLER_DIGITAL_R2, [=] () -> void {
                intakeStateController.setCurrentBehavior(intakeEject.until([=] () -> bool {
                    return !controller1->get_digital(E_CONTROLLER_DIGITAL_R2);}));});
		}

		void update() override {

			if (controller1->get_digital_new_press(E_CONTROLLER_DIGITAL_R1)) {
				intakeExtensionStateController.setCurrentBehavior(&intakeIntaking);
			}
			if (!controller1->get_digital(E_CONTROLLER_DIGITAL_R1) && intakeStateController.getCurrentBehavior() == &intakeIntaking) {
				intakeStateController.setCurrentBehavior(hasTriball ? &intakeHold : &intakeStopped);
			}
		}

		void exit() override {
		}

		bool isDone() override {
			return false;
		}

		~TeleopModeLogic() {}
	};
} // namespace Pronounce
