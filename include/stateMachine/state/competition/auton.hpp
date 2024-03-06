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

		void setAuton(pros::task_fn_t auton) {
			Log("Set auton");
			this->auton = auton;
		}

		void initialize() override {
			Log("Init");
			Enabled::initialize();

			drivetrainStateController->setDefaultBehavior(drivetrainStopped);
			drivetrainStateController->useDefaultBehavior();
			drivetrain.setBrakeMode(pros::MotorBrake::hold);

			if (task.get_state() == 2)
				task.remove();

			task = pros::Task(auton, nullptr, TASK_PRIORITY_MAX, TASK_STACK_DEPTH_DEFAULT, "User auton");
		}

		void update() override {
			Log("Update");
			Enabled::update();
		}

		void exit() override {
			Log("Exit");
			if (task.get_state() == 2)
				task.remove();

			drivetrainStateController->ud();
			intakeStateController->ud();

			Enabled::exit();
		}

		bool isDone() override {
			return task.get_state() == 4;
		}
	};

} // Pronounce
