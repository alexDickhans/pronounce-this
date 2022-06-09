#pragma once

#include "launcher.hpp"
#include "feedbackControllers/flywheelPID.hpp"
#include "api.h"
#include "utils/motorGroup.hpp"
#include "stateMachine/wait.hpp"
#include "stateMachine/stateController.hpp"
#include "stateMachine/behavior.hpp"

namespace Pronounce {

	pros::Motor flywheel1(1, pros::E_MOTOR_GEARSET_36, true);
	pros::Motor flywheel2(2, pros::E_MOTOR_GEARSET_36, false);

	pros::Motor turretMotor(3, false);

	MotorGroup flywheels;

	pros::ADIDigitalOut indexer(1, false);

	FlywheelPID flywheelPID(3.0, 0.5, 0.0, 3.0);
	PID turretPID(600.0, 0.0, 100);

	Launcher launcherStopped(0.0, 42.0 / 1.0, false, false, &flywheels, &turretMotor, &indexer, &flywheelPID, &turretPID);
	Launcher launcherIdle(1.0, 42.0 / 1.0, false, false, &flywheels, &turretMotor, &indexer, &flywheelPID, &turretPID);
	Launcher launcherFullSpeed(1.0, 42.0 / 1.0, false, true, &flywheels, &turretMotor, &indexer, &flywheelPID, &turretPID);
	Launcher launcherLaunching(1.0, 42.0 / 1.0, true, false, &flywheels, &turretMotor, &indexer, &flywheelPID, &turretPID);

	StateController launcherStateController(&launcherIdle);
	StateController launcherStateExtensionController(new Behavior());

	Wait launchDisc1(&launcherLaunching, 300);
	Wait launchDisc2(&launcherFullSpeed, 700);

	Sequence launchDisc;

	void initLauncherStates() {
		flywheelPID.setMaxIntegral(4000.0);
		flywheelPID.setIntegralBound(4000.0);

		flywheels.addMotor(&flywheel1);
		flywheels.addMotor(&flywheel2);

		launchDisc.addState(&launcherStateController, &launcherFullSpeed);
		launchDisc.addState(&launcherStateController, &launchDisc1);

		turretMotor.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
	}
} // namespace Pronounce
