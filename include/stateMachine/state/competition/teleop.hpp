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
		AbstractJoystick& controller1;
	public:
		explicit Teleop(AbstractJoystick& controller1) : Enabled("Teleop"), controller1(controller1) {
		}

		void initialize() override {
			Log("Init");
			drivetrainStateController->setDefaultBehavior(normalJoystick);
			drivetrainStateController->ud();

			drivetrain.setBrakeMode(pros::MotorBrake::coast);
			intakeMotors.set_brake_mode_all(pros::E_MOTOR_BRAKE_COAST);

			controller1.clearCallbacks();

			controller1.onPressed(E_CONTROLLER_DIGITAL_L2, [&]() -> void {
				if (controller1.get_digital(E_CONTROLLER_DIGITAL_Y)) {
					winchStateExtensionController->sb(winchASequence);
				} else {
					leftWingStateController->sb(std::make_shared<Until>(leftWingOut, [=, this]() -> bool {
						return !controller1.get_digital(E_CONTROLLER_DIGITAL_L2);
					}));
				}
			});

			controller1.onPressed(E_CONTROLLER_DIGITAL_L1, [&]() -> void {
				rightWingStateController->sb(std::make_shared<Until>(rightWingOut, [&]() -> bool {
					return !controller1.get_digital(E_CONTROLLER_DIGITAL_L1);
				}));
			});

			controller1.onPressed(E_CONTROLLER_DIGITAL_R2, [&]() -> void {
				if (controller1.get_digital(E_CONTROLLER_DIGITAL_Y)) {
					winchStateExtensionController->sb(winchCSequence);
				} else {
					intakeStateController->sb(std::make_shared<Until>(intakeEject, [&]() -> bool {
						return !controller1.get_digital(E_CONTROLLER_DIGITAL_R2);
					}));
				}
			});

			controller1.onPressed(E_CONTROLLER_DIGITAL_R1, [&]() -> void {
				intakeStateController->sb(std::make_shared<Until>(intakeIntaking, [&]() -> bool {
					return !controller1.get_digital(E_CONTROLLER_DIGITAL_R1);
				}));
			});

			controller1.onPressed(E_CONTROLLER_DIGITAL_Y, [&]() -> void {
					if (controller1.get_digital(E_CONTROLLER_DIGITAL_RIGHT)) {
						winchStateController->sb(winchUpCheese);
					}
			});

			controller1.onPressed(E_CONTROLLER_DIGITAL_RIGHT, [&]() -> void {
				if (controller1.get_digital(E_CONTROLLER_DIGITAL_Y)) {
					winchStateController->sb(winchUpCheese);
				}
			});

			Enabled::initialize();
		}

		void update() override {
			Log("Update");
			Enabled::update();

			controller1.update();
		}

		void exit() override {
			Log("Exit");
			drivetrainStateController->setDefaultBehavior(drivetrainStopped);
			leftWingStateController->ud();
			rightWingStateController->ud();
			drivetrainStateController->ud();
			controller1.clearCallbacks();
			Enabled::exit();
		}

		bool isDone() override {
			return false;
		}

		~Teleop() = default;
	};
} // namespace Pronounce