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

		pathFollower->addCommandMapping("frontLeftWingOut", [&]() -> void {
			frontLeftWingStateController->sb(frontLeftWingOut);
		});

		pathFollower->addCommandMapping("frontLeftWingIn", [&]() -> void {
			frontLeftWingStateController->sb(frontLeftWingIn);
		});

		pathFollower->addCommandMapping("frontRightWingOut", [&]() -> void {
			frontRightWingStateController->sb(frontRightWingOut);
		});

		pathFollower->addCommandMapping("frontRightWingIn", [&]() -> void {
			frontRightWingStateController->sb(frontRightWingIn);
		});

		pathFollower->addCommandMapping("backRightWingOut", [&]() -> void {
			backRightWingStateController->sb(backRightWingOut);
		});

		pathFollower->addCommandMapping("backRightWingIn", [&]() -> void {
			backRightWingStateController->sb(backRightWingIn);
		});

		pathFollower->addCommandMapping("backLeftWingOut", [&]() -> void {
			backLeftWingStateController->sb(backLeftWingOut);
		});

		pathFollower->addCommandMapping("backLeftWingIn", [&]() -> void {
			backLeftWingStateController->sb(backLeftWingIn);
		});

		pathFollower->addCommandMapping("frontWingsOut", [&]() -> void {
			frontLeftWingStateController->sb(frontLeftWingOut);
			frontRightWingStateController->sb(frontRightWingOut);
		});

		pathFollower->addCommandMapping("frontWingsIn", [&]() -> void {
			frontLeftWingStateController->sb(frontLeftWingIn);
			frontRightWingStateController->sb(frontRightWingIn);
		});

		pathFollower->addCommandMapping("backWingsOut", [&]() -> void {
			backLeftWingStateController->sb(backLeftWingOut);
			backRightWingStateController->sb(backRightWingOut);
		});

		pathFollower->addCommandMapping("backWingsIn", [&]() -> void {
			backLeftWingStateController->sb(backLeftWingIn);
			backRightWingStateController->sb(backRightWingIn);
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
