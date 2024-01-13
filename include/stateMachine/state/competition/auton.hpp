#pragma once

#include "stateMachine/behavior.hpp"
#include "hardware/hardware.hpp"
#include "enabled.hpp"

namespace Pronounce {

	class Auton : public Enabled {
	private:
		pros::task_fn_t auton;
		pros::Task task;
	public:
		Auton(pros::task_fn_t auton) : task([=]() -> void { return;}), Enabled("Auton") {
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

			if (task.get_state() == 2)
				task.remove();

			task = pros::Task(auton, nullptr, TASK_PRIORITY_MAX, TASK_STACK_DEPTH_DEFAULT, "User auton");
		}

		void update() override {
			Enabled::update();
		}

		void exit() override {
			if (task.get_state() == 2)
				task.remove();

			drivetrainStateController.useDefaultBehavior();
			intakeStateController.useDefaultBehavior();
			catapultStateController.useDefaultBehavior();

			Enabled::exit();
		}

		bool isDone() override {
			return task.get_state() == 4;
		}
	};

} // Pronounce
