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

	pros::Motor flywheel1(11, pros::E_MOTOR_GEARSET_36, false);

	pros::Motor turretMotor(3, pros::E_MOTOR_GEARSET_06, false);

	MotorGroup flywheels;

	pros::ADIDigitalOut indexer(1, false);

	FlywheelPID flywheelPID(10.0, 0.0, -8.0, 3.2);
	PID turretPID(30000.0, 0.0, 35000.0);

	pros::Rotation turretRotation(4);

	Launcher launcherStopped(0.0, 36.0 / 1.0, false, false, &flywheels, &turretMotor, &indexer, &flywheelPID, &turretPID, turretRotation);
	Launcher launcherIdle(1.0, 36.0 / 1.0, false, false, &flywheels, &turretMotor, &indexer, &flywheelPID, &turretPID, turretRotation);
	Launcher launcherFullSpeed(1.0, 36.0 / 1.0, false, true, &flywheels, &turretMotor, &indexer, &flywheelPID, &turretPID, turretRotation);
	Launcher launcherLaunching(1.0, 36.0 / 1.0, true, false, &flywheels, &turretMotor, &indexer, &flywheelPID, &turretPID, turretRotation);

	StateController launcherStateController(&launcherIdle);
	StateController launcherStateExtensionController(new Behavior());

	Wait launchDisc1(&launcherLaunching, 300);
	Wait launchDisc2(&launcherFullSpeed, 700);

	Sequence launchDisc;

	pros::Vision turretVision(5, VISION_ZERO_CENTER);

	LinearInterpolator flywheelRPM;

	const uint8_t RED_GOAL = 1;

	void initLauncherStates() {
		flywheelPID.setMaxIntegral(4000.0);
		flywheelPID.setIntegralBound(4000.0);

		flywheels.addMotor(&flywheel1);

		launchDisc.addState(&launcherStateController, &launcherFullSpeed);
		launchDisc.addState(&launcherStateController, &launchDisc1);

		turretMotor.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);

		pros::vision_signature_s_t redGoal = pros::Vision::signature_from_utility(RED_GOAL, 4979, 7501, 6240, -601, 307, -147, 3.000, 0);
		turretVision.set_signature(RED_GOAL, &redGoal);	
		turretVision.set_exposure(95);

		flywheelRPM.add(38.0, 2135.0);
		flywheelRPM.add(62.0, 2200.0);
		flywheelRPM.add(86.0, 2410.0);
		flywheelRPM.add(110.0, 2750.0);
		flywheelRPM.add(134.0, 2800.0);
	}
} // namespace Pronounce
