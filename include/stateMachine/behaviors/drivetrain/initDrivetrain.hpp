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
#include "motionControl/ramsete.hpp"

// TODO: Add comments

namespace Pronounce {

	PID turningPid(0.30, 0.0, 1.1, 0.0, 0.0, true);
	PID visionPid(0.0045, 0.0, 0.050);

	// Drivetrain states for driving around the field and shooting at the goal
	JoystickDrivetrain normalJoystick("NormalJoystick", odometry, master, drivetrain, visionPid, 0.10, false, 2.4, 73_in / second);
	JoystickDrivetrain targetingJoystick("TargetingJoystick", odometry, master, drivetrain, visionPid, 0.10, true, 2.4, 73_in / second);

	JoystickDrivetrain drivetrainStopped("DrivetrainStopped", odometry, master, drivetrain, visionPid, 0.10, true, 2.4, 0.0);

	StateController drivetrainStateController("DrivetrainStateController", &drivetrainStopped);

	ProfileConstraints defaultProfileConstraints = { 60_in / second, 150_in / second / second, 0.0 };
	ProfileConstraints intakeProfileConstraints = { 20_in / second, 200_in / second / second, 0.0 };
	ProfileConstraints stackIntakeProfileConstraints = { 10_in / second, 200_in / second / second, 0.0 };

	// Autonomous states

	// Motion Controllers
	TankMotionProfiling moveForward5in("MoveForward5in", &drivetrain, intakeProfileConstraints, 8_in, &odometry, drivetrainMutex, 0.0);
	TankMotionProfiling moveBackward5in("MoveBackward5in", &drivetrain, defaultProfileConstraints, -5_in, &odometry, drivetrainMutex, 0.0);
	TankMotionProfiling testCurvatureDrive("TestCurvatureDrive", &drivetrain, intakeProfileConstraints, 45_in, &odometry, drivetrainMutex, (90_deg / 20_in));

	TankMotionProfiling intakeDiscStack("IntakeDiscStack", &drivetrain, stackIntakeProfileConstraints, 20_in, &odometry, drivetrainMutex, 0.0);
	TankMotionProfiling intakeBarrierDiscs("IntakeBarrierDiscs", &drivetrain, defaultProfileConstraints, 20_in, &odometry, drivetrainMutex, 0.0);

	// Skills
	TankMotionProfiling intoCloseRoller2("IntoCloseRoller2", &drivetrain, intakeProfileConstraints, 5_in, &odometry, drivetrainMutex, 0.0/*180_deg/-4_in*/);
	TankMotionProfiling fromCloseRoller2("FromCloseRoller", &drivetrain, defaultProfileConstraints, -30_in, &odometry, drivetrainMutex, (80_deg/-12_in));
	TankMotionProfiling closeRollerToLeftRoller("CloseRollerToLeftRoller", &drivetrain, defaultProfileConstraints, 38_in, &odometry, drivetrainMutex, 0.0);
	TankMotionProfiling intoLeftRoller("ToCloseRoller", &drivetrain, stackIntakeProfileConstraints, 3_in, &odometry, drivetrainMutex, (8_deg/-10_in));
	TankMotionProfiling fromLeftRoller("FromLeftRoller", &drivetrain, defaultProfileConstraints, -8_in, &odometry, drivetrainMutex, (130_deg/-10_in));
	TankMotionProfiling leftRollerToGoal("RollerToGoal", &drivetrain, defaultProfileConstraints, 67_in, &odometry, drivetrainMutex, (20_deg/85_in));
	TankMotionProfiling intakeFarBarrierCloseDiscs("IntakeFarBarrierCloseDiscs", &drivetrain, intakeProfileConstraints, 35_in, &odometry, drivetrainMutex, 0.0);
	TankMotionProfiling backIntakeFarBarrierCloseDiscs("IntakeFarBarrierCloseDiscs", &drivetrain, defaultProfileConstraints, -35_in, &odometry, drivetrainMutex, 0.0);
	TankMotionProfiling toMiddleLineDiscs("ToMiddleLineDiscs", &drivetrain, defaultProfileConstraints, 35_in, &odometry, drivetrainMutex, 0.0);
	TankMotionProfiling intakeMiddleLineDiscs("IntakeMiddleLineDiscs", &drivetrain, intakeProfileConstraints, 35_in, &odometry, drivetrainMutex, 0.0);
	TankMotionProfiling toFarBarrier("toFarBarrier", &drivetrain, intakeProfileConstraints, 8_in, &odometry, drivetrainMutex, 0.0);
	TankMotionProfiling intakeFarBarrierFarDiscs("IntakeFarBarrierCloseDiscs", &drivetrain, intakeProfileConstraints, 42_in, &odometry, drivetrainMutex, 0.0);

	// Rotation Controllers
	RotationController turnTo0("turnTo0", drivetrain, odometry, turningPid, 0_deg, drivetrainMutex);
	RotationController turnTo10("turnTo10", drivetrain, odometry, turningPid, 10_deg, drivetrainMutex);
	RotationController turnTo35("turnTo35", drivetrain, odometry, turningPid, 350_deg, drivetrainMutex);
	RotationController turnTo45("turnTo45", drivetrain, odometry, turningPid, 50_deg, drivetrainMutex);
	RotationController turnTo65("turnTo65", drivetrain, odometry, turningPid, 75_deg, drivetrainMutex);
	RotationController turnTo70("turnTo70", drivetrain, odometry, turningPid, 67_deg, drivetrainMutex);
	RotationController turnTo90("turnTo90", drivetrain, odometry, turningPid, 90_deg, drivetrainMutex);
	RotationController turnTo115("turnTo115", drivetrain, odometry, turningPid, 120_deg, drivetrainMutex);
	RotationController turnTo125("turnTo125", drivetrain, odometry, turningPid, 128_deg, drivetrainMutex);
	RotationController turnTo135("turnTo135", drivetrain, odometry, turningPid, 135_deg, drivetrainMutex);
	RotationController turnTo145("turnTo145", drivetrain, odometry, turningPid, 145_deg, drivetrainMutex);
	RotationController turnTo165("turnTo175", drivetrain, odometry, turningPid, 170_deg, drivetrainMutex);
	RotationController turnTo175("turnTo175", drivetrain, odometry, turningPid, 175_deg, drivetrainMutex);
	RotationController turnTo180("turnTo180", drivetrain, odometry, turningPid, 180_deg, drivetrainMutex);
	RotationController turnTo225("turnTo225", drivetrain, odometry, turningPid, 225_deg, drivetrainMutex);
	RotationController turnTo270("turnTo270", drivetrain, odometry, turningPid, 270_deg, drivetrainMutex);
	RotationController turnTo315("TurnTo315", drivetrain, odometry, turningPid, 315_deg, drivetrainMutex);
	RotationController turnTo325("TurnTo325", drivetrain, odometry, turningPid, 325_deg, drivetrainMutex);
	RotationController turnTo335("TurnTo335", drivetrain, odometry, turningPid, 335_deg, drivetrainMutex);
	RotationController turnTo305("TurnTo305", drivetrain, odometry, turningPid, 305_deg, drivetrainMutex);
	RotationController turnTo350("TurnTo305", drivetrain, odometry, turningPid, 350_deg, drivetrainMutex);

	RamseteController testRamsete(&drivetrain, &odometry, intakeProfileConstraints, Pose2D(0_in, 24_in, 0_deg), 0.0, 0.0);

	BezierPath testPath("TestPath");

	TankPurePursuit testPathPurePursuit("TestPurePursuit", &drivetrain, &odometry, { 5_in, SinusoidalVelocityProfile(10_in, intakeProfileConstraints) }, testPath);

	void initDrivetrain() {

		testPath.addPoint(SplinePoint(Point(0.0, 0.0), Vector(5_in, 0.0)));
		testPath.addPoint(SplinePoint(Point(24_in, 24_in), Vector(5_in, 0.0)));

		testPathPurePursuit.setPath(testPath.getPath(0.01));
	}
} // namespace Pronounce
