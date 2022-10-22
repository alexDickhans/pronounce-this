#pragma once

#include "joystick.hpp"
#include "api.h"
#include "stateMachine/wait.hpp"
#include "stateMachine/stateController.hpp"
#include "stateMachine/behavior.hpp"
#include "odometry/continuousOdometry/continuousOdometry.hpp"
#include "position/trackingWheel.hpp"
#include "chassis/xdrive.hpp"
#include "utils/runningAverage.hpp"
#include "odometry/odomFuser.hpp"
#include "motionControl/omniMotionProfiling.hpp"
#include "velocityProfile/sinusoidalVelocityProfile.hpp"
#include "utils/path.hpp"
#include "utils/quadraticSplinePath.hpp"
#include "motionControl/rotationController.hpp"
#include "hardware/hardware.hpp"
#include "motionControl/tankMotionProfiling.hpp"
#include "chassis/tankDrive.hpp"

// TODO: Add comments

namespace Pronounce {
	
	std::shared_ptr<PID> turningPid(new PID(330, 0, 2000));
	std::shared_ptr<PID> visionPid(new PID(400, 50, 1000));

	// Drivetrain states for driving around the field and shooting at the goal
	JoystickDrivetrain normalJoystick("NormalJoystick", odometry, master, drivetrain, visionPid, 0.10, false, 2.4, 73_in / second);
	JoystickDrivetrain targetingJoystick("TargetingJoystick", odometry, master, drivetrain, visionPid, 0.10, true, 2.4, 73_in / second);

	JoystickDrivetrain drivetrainStopped("DrivetrainStopped", odometry, master, drivetrain, visionPid, 0.10, true, 2.4, 0.0);

	StateController drivetrainStateController("DrivetrainStateController", &drivetrainStopped);

	ProfileConstraints defaultProfileConstraints = {70_in/second, 100_in/second/second, 0.0};
	ProfileConstraints intakeProfileConstraints = {40_in/second, 100_in/second/second, 0.0};

	// Autonomous states

	// Motion Controllers
	TankMotionProfiling moveForward5in("MoveForward5in", &drivetrain, intakeProfileConstraints, 5_in, &odometry, drivetrainMutex);
	TankMotionProfiling moveBackward5in("MoveBackward5in", &drivetrain, intakeProfileConstraints, -5_in, &odometry, drivetrainMutex);
	TankMotionProfiling closeToMidField("CloseToMidField", &drivetrain, defaultProfileConstraints, -60_in, &odometry, drivetrainMutex);
	TankMotionProfiling midFieldToAutonLine("MidFieldToAutonLine", &drivetrain, defaultProfileConstraints, -10_in, &odometry, drivetrainMutex);
	TankMotionProfiling autonLineMidField("AutonLineMidField", &drivetrain, intakeProfileConstraints, 10_in, &odometry, drivetrainMutex);
	TankMotionProfiling autonLineToMidDiscs("AutonLineToMidDiscs", &drivetrain, intakeProfileConstraints, -20_in, &odometry, drivetrainMutex);
	TankMotionProfiling midDiscsToAutonLine("MidDiscsToAutonLine", &drivetrain, defaultProfileConstraints, 20_in, &odometry, drivetrainMutex);
	TankMotionProfiling midFieldToFarField("midFieldToFarField", &drivetrain, defaultProfileConstraints, 60_in, &odometry, drivetrainMutex);

	// Rotation Controllers
	RotationController turnTo0("turnTo0", drivetrain, odometry, (*turningPid), 0_deg, drivetrainMutex);
	RotationController turnTo45("turnTo45", drivetrain, odometry, (*turningPid), 45_deg, drivetrainMutex);
	RotationController turnTo90("turnTo90", drivetrain, odometry, (*turningPid), 90_deg, drivetrainMutex);
	RotationController turnTo135("turnTo135", drivetrain, odometry, (*turningPid), 135_deg, drivetrainMutex);
	RotationController turnTo180("turnTo180", drivetrain, odometry, (*turningPid), 180_deg, drivetrainMutex);
	RotationController turnTo225("turnTo225", drivetrain, odometry, (*turningPid), 225_deg, drivetrainMutex);
	RotationController turnTo270("turnTo270", drivetrain, odometry, (*turningPid), 270_deg, drivetrainMutex);
	RotationController turnTo315("TurnTo315", drivetrain, odometry, (*turningPid), 315_deg, drivetrainMutex);

	RotationController turnTo325("TurnTo325", drivetrain, odometry, (*turningPid), 325_deg, drivetrainMutex);
	RotationController turnTo305("TurnTo305", drivetrain, odometry, (*turningPid), 305_deg, drivetrainMutex);

	void initDrivetrain() {
		leftDriveMotors.set_brake_modes(MOTOR_BRAKE_HOLD);
		rightDriveMotors.set_brake_modes(MOTOR_BRAKE_HOLD);
	}
} // namespace Pronounce
