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

				leftWingStateController.setCurrentBehavior(leftWingOut.until([=] () -> bool {
					return !controller1->get_digital(E_CONTROLLER_DIGITAL_L2);
				}));
				rightWingStateController.setCurrentBehavior(rightWingOut.until([&] () -> bool {
					return !controller1->get_digital(E_CONTROLLER_DIGITAL_L2);
				}));
			});

			controller1->onPressed(E_CONTROLLER_DIGITAL_R2, [&] () -> void {
				intakeExtensionStateController.useDefaultBehavior();
				intakeStateController.setCurrentBehavior(intakeEject.until([=] () -> bool {
					return !controller1->get_digital(E_CONTROLLER_DIGITAL_R2);}));
			});

			controller1->onPressed(E_CONTROLLER_DIGITAL_R1, [=] () -> void {
				intakeExtensionStateController.setCurrentBehavior(&intakeSequence);
			});

			controller1->onPressed(E_CONTROLLER_DIGITAL_L1, [=] () -> void {
				awpStateController.setCurrentBehavior(awpOut.until([&] () -> bool {
					return !controller1->get_digital(E_CONTROLLER_DIGITAL_L1);
				}));
			});

			Enabled::initialize();
		}

		void update() override {
			Enabled::update();

			controller1->update();
			controller2->update();
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