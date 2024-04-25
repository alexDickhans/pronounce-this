#pragma once

#include "stateMachine/behavior.hpp"
#include "hardware/hardware.hpp"
#include "enabled.hpp"

namespace Pronounce {

	void initAutonomousMappings() {
		pathFollower->addCommandMapping("intake", [&]() -> void {
			intakeStateController->sb(intakeIntaking);
		});

		pathFollower->addCommandMapping("intakeStop", [&]() -> void {
			intakeStateController->ud();
		});

		pathFollower->addCommandMapping("outtake", [&]() -> void {
			intakeStateController->sb(intakeEject);
		});

		pathFollower->addCommandMapping("leftWingOut", [&]() -> void {
			leftWingStateController->sb(leftWingOut);
		});

		pathFollower->addCommandMapping("leftWingIn", [&]() -> void {
			leftWingStateController->sb(leftWingIn);
		});

		pathFollower->addCommandMapping("rightWingOut", [&]() -> void {
			rightWingStateController->sb(rightWingOut);
		});

		pathFollower->addCommandMapping("rightWingIn", [&]() -> void {
			rightWingStateController->sb(rightWingIn);
		});

		pathFollower->addCommandMapping("frontWingsOut", [&]() -> void {
			leftWingStateController->sb(leftWingOut);
			rightWingStateController->sb(rightWingOut);
		});

		pathFollower->addCommandMapping("frontWingsIn", [&]() -> void {
			leftWingStateController->sb(leftWingIn);
			rightWingStateController->sb(rightWingIn);
		});

		pathFollower->addCommandMapping("hangOut", [&]() -> void {
			winchStateController->sb(winchUp);
		});

		pathFollower->addCommandMapping("hangIn", [&]() -> void {
			winchStateController->sb(winchC);
		});
	}

	class Auton : public Enabled {
	private:
		pros::task_fn_t auton;
		void* args;
		pros::Task task;
	public:
		Auton(pros::task_fn_t auton, void* args = nullptr) : task([=]() -> void { return; }), Enabled("Auton") {
			this->auton = auton;
			this->args = args;
		}

		void setAuton(pros::task_fn_t auton, void* args = nullptr) {
			Log("Set auton");
			this->auton = auton;
			this->args = args;
		}

		void initialize() override {
			Log("Init");
			drivetrainStateController->setDefaultBehavior(drivetrainStopped);
			drivetrainStateController->useDefaultBehavior();
			Enabled::initialize();

			if (task.get_state() == 2)
				task.remove();

			task = pros::Task(auton, args, TASK_PRIORITY_MAX-1, TASK_STACK_DEPTH_DEFAULT, "User auton");
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
