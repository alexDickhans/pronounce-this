#pragma once

#include "stateMachine/behavior.hpp"
#include "hardware/hardware.hpp"
#include "enabled.hpp"

namespace Pronounce {

	class Auton : public Enabled {
	private:
		pros::task_fn_t auton;
		pros::task_t task;
	public:
		Auton(pros::task_fn_t auton) : Enabled("Auton") {
			this->auton = auton;
		}

		Auton* setAuton(pros::task_fn_t auton) {
			this->auton = auton;
			return this;
		}

		void initialize() override {
			Enabled::initialize();

			drivetrainStateController.setDefaultBehavior(&drivetrainStopped);
			drivetrainStateController.useDefaultBehavior();
			leftDriveMotors.set_brake_modes(pros::E_MOTOR_BRAKE_HOLD);
			rightDriveMotors.set_brake_modes(pros::E_MOTOR_BRAKE_HOLD);

			task = pros::c::task_create(auton, (void*) NULL, TASK_PRIORITY_MAX, TASK_STACK_DEPTH_DEFAULT, "User auton");
		}

		void exit() override {
			pros::c::task_delete(task);

			drivetrainStateController.useDefaultBehavior();
			intakeStateController.useDefaultBehavior();
			catapultStateController.useDefaultBehavior();
			leftWingStateController.useDefaultBehavior();
			rightWingStateController.useDefaultBehavior();

			Enabled::exit();
		}

		bool isDone() {
			return pros::c::task_get_state(task) == pros::E_TASK_STATE_RUNNING;
		}
	};

} // Pronounce
