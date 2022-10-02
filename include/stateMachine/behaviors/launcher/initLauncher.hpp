#pragma once

#include "launcher.hpp"
#include "feedbackControllers/flywheelPID.hpp"
#include "api.h"
#include "utils/motorGroup.hpp"
#include "stateMachine/wait.hpp"
#include "stateMachine/stateController.hpp"
#include "stateMachine/behavior.hpp"
#include "utils/linearInterpolator.hpp"

// TODO: Clean up
// TODO: move declarations to another place
// TODO: Add comments

namespace Pronounce {

	pros::Motor flywheel1(16, pros::E_MOTOR_GEARSET_36, true);

	pros::Motor turretMotor(3, pros::E_MOTOR_GEARSET_06, false);

	MotorGroup flywheels;

	pros::ADIDigitalOut indexer(1, false);

	FlywheelPID flywheelPID(4.0, 0.1, 0.0, 3.55);
	PID turretPID(30000.0, 0.0, 120000.0);

	pros::Rotation turretRotation(4);

	Launcher launcherStopped("LauncherStopped", 0.0, 30.0 / 1.0, false, false, &flywheels, &turretMotor, &indexer, &flywheelPID, &turretPID, turretRotation);
	Launcher launcherIdle("LauncherIdle", 1.0, 30.0 / 1.0, false, false, &flywheels, &turretMotor, &indexer, &flywheelPID, &turretPID, turretRotation);
	Launcher launcherFullSpeed("LauncherFullSpeed", 1.0, 30.0 / 1.0, false, true, &flywheels, &turretMotor, &indexer, &flywheelPID, &turretPID, turretRotation);
	Launcher launcherLaunching("LauncherLaunching", 1.0, 30.0 / 1.0, true, false, &flywheels, &turretMotor, &indexer, &flywheelPID, &turretPID, turretRotation);

	StateController launcherStateController("LauncherStateController", &launcherIdle);
	StateController launcherStateExtensionController("LauncherStateExtensionController", new Behavior());

	Wait launchDisc1(&launcherLaunching, 300_ms);
	Wait launchDisc2(&launcherFullSpeed, 1500_ms);

	Sequence launchDisc("LaunchDisc");

	Sequence launch2Disc("Launch2Disc");

	Sequence launch3Disc("Launch3Disc");

	LinearInterpolator flywheelRPM;

	const uint8_t RED_GOAL = 1;

	void initLauncherStates() {
		flywheelPID.setMaxIntegral(4000.0);
		flywheelPID.setIntegralBound(4000.0);

		flywheels.addMotor(&flywheel1);

		launchDisc.addState(&launcherStateController, &launcherFullSpeed);
		launchDisc.addState(&launcherStateController, &launchDisc1);

		launch2Disc.addState(&launcherStateController, &launchDisc2);
		launch2Disc.addState(&launcherStateController, &launchDisc1);
		launch2Disc.addState(&launcherStateController, &launchDisc2);
		launch2Disc.addState(&launcherStateController, &launchDisc1);

		launch3Disc.addState(&launcherStateController, &launchDisc1);
		launch3Disc.addState(&launcherStateController, &launchDisc2);

		turretMotor.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);

		// pros::vision_signature_s_t redGoal = pros::Vision::signature_from_utility(RED_GOAL, 4979, 7501, 6240, -601, 307, -147, 3.000, 0);
		// turretVision.set_signature(RED_GOAL, &redGoal);	
		// turretVision.set_exposure(95);

		flywheelRPM.add(38.0, 2335.0);
		flywheelRPM.add(62.0, 2400.0);
		flywheelRPM.add(86.0, 2700.0);
		flywheelRPM.add(110.0, 2900.0);
		flywheelRPM.add(134.0, 3200.0);
	}
} // namespace Pronounce
