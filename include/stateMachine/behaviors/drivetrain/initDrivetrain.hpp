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
	
	PID turningPid(0.30, 0, 0.6, 0.0, 0.0, true);
	PID visionPid(0.004, 0, 0.08);

	// Drivetrain states for driving around the field and shooting at the goal
	JoystickDrivetrain normalJoystick("NormalJoystick", odometry, master, drivetrain, visionPid, 0.10, false, 2.4, 73_in / second);
	JoystickDrivetrain targetingJoystick("TargetingJoystick", odometry, master, drivetrain, visionPid, 0.10, true, 2.4, 73_in / second);

	JoystickDrivetrain drivetrainStopped("DrivetrainStopped", odometry, master, drivetrain, visionPid, 0.10, true, 2.4, 0.0);

	StateController drivetrainStateController("DrivetrainStateController", &drivetrainStopped);

	ProfileConstraints defaultProfileConstraints = {70_in/second, 200_in/second/second, 0.0};
	ProfileConstraints intakeProfileConstraints = {30_in/second, 200_in/second/second, 0.0};
	ProfileConstraints stackIntakeProfileConstraints = {5_in/second, 200_in/second/second, 0.0};

	// Autonomous states

	// Motion Controllers
	TankMotionProfiling moveForward5in("MoveForward5in", &drivetrain, defaultProfileConstraints, 5_in, &odometry, drivetrainMutex);
	TankMotionProfiling moveBackward5in("MoveBackward5in", &drivetrain, defaultProfileConstraints, -5_in, &odometry, drivetrainMutex);
	TankMotionProfiling closeToMidField("CloseToMidField", &drivetrain, defaultProfileConstraints, -72_in, &odometry, drivetrainMutex);
	TankMotionProfiling midFieldToAutonLine("MidFieldToAutonLine", &drivetrain, defaultProfileConstraints, -10_in, &odometry, drivetrainMutex);
	TankMotionProfiling autonLineMidField("AutonLineMidField", &drivetrain, intakeProfileConstraints, 10_in, &odometry, drivetrainMutex);
	TankMotionProfiling autonLineToMidDiscs("AutonLineToMidDiscs", &drivetrain, intakeProfileConstraints, 20_in, &odometry, drivetrainMutex);
	TankMotionProfiling midDiscsToAutonLine("MidDiscsToAutonLine", &drivetrain, defaultProfileConstraints, -20_in, &odometry, drivetrainMutex);
	TankMotionProfiling midFieldToFarField("midFieldToFarField", &drivetrain, defaultProfileConstraints, 76_in, &odometry, drivetrainMutex);

	TankMotionProfiling backwards48in("Backwards48in", &drivetrain, defaultProfileConstraints, -48_in, &odometry, drivetrainMutex);

	// SKILLS
	TankMotionProfiling closeRollerToBottomLeftCorner("CloseRollerToBottomLeftCorner", &drivetrain, defaultProfileConstraints, 19_in, &odometry, drivetrainMutex);
	TankMotionProfiling leftRollerToGoal("LeftRollerToGoal", &drivetrain, defaultProfileConstraints, -72_in, &odometry, drivetrainMutex);
	TankMotionProfiling leftGoalToMiddleDiscs("LeftGoalToMiddleDiscs", &drivetrain, defaultProfileConstraints, 48_in, &odometry, drivetrainMutex);
	TankMotionProfiling collectMiddleDiscs("CollectMiddleDiscs", &drivetrain, intakeProfileConstraints, 40_in, &odometry, drivetrainMutex);
	TankMotionProfiling middleDiscsToTopGoal("MiddleDiscsToTopGoal", &drivetrain, defaultProfileConstraints, 42_in, &odometry, drivetrainMutex);
	TankMotionProfiling toTopGoal("ToTopGoal", &drivetrain, defaultProfileConstraints, -15_in, &odometry, drivetrainMutex);
	TankMotionProfiling middleStackToTopGoal("MiddleStackToTopGoal", &drivetrain, defaultProfileConstraints, -40_in, &odometry, drivetrainMutex);
	TankMotionProfiling intakeMiddleStack("IntakeMiddleStack", &drivetrain, stackIntakeProfileConstraints, 15_in, &odometry, drivetrainMutex);
	TankMotionProfiling leftGoalToMiddleStack("LeftGoalToMiddleStack", &drivetrain, defaultProfileConstraints, 25_in, &odometry, drivetrainMutex);
	TankMotionProfiling topGoalToTopMiddleStack("TopGoalToTopMiddleStack", &drivetrain, defaultProfileConstraints, 45_in, &odometry, drivetrainMutex);
	TankMotionProfiling topMiddleStackToTopGoal("TopMiddleStackToTopGoal", &drivetrain, defaultProfileConstraints, -60_in, &odometry, drivetrainMutex);
	TankMotionProfiling moveToTopAutonLineDiscs("MoveToTopAutonLineDiscs", &drivetrain, defaultProfileConstraints, 65_in, &odometry, drivetrainMutex);
	TankMotionProfiling intakeAutonLineDiscs("IntakeAutonLineDiscs", &drivetrain, defaultProfileConstraints, 35_in, &odometry, drivetrainMutex);
	TankMotionProfiling autonLineToLeftRoller("AutonLineToLeftRoller", &drivetrain, defaultProfileConstraints, 35_in, &odometry, drivetrainMutex);
	
	// Rotation Controllers
	RotationController turnTo0("turnTo0", drivetrain, odometry, turningPid, 0_deg, drivetrainMutex);
	RotationController turnTo45("turnTo45", drivetrain, odometry, turningPid, 46_deg, drivetrainMutex);
	RotationController turnTo90("turnTo90", drivetrain, odometry, turningPid, 90_deg, drivetrainMutex);
	RotationController turnTo115("turnTo115", drivetrain, odometry, turningPid, 115_deg, drivetrainMutex);
	RotationController turnTo125("turnTo125", drivetrain, odometry, turningPid, 125_deg, drivetrainMutex);
	RotationController turnTo135("turnTo135", drivetrain, odometry, turningPid, 135_deg, drivetrainMutex);
	RotationController turnTo145("turnTo145", drivetrain, odometry, turningPid, 145_deg, drivetrainMutex);
	RotationController turnTo180("turnTo180", drivetrain, odometry, turningPid, 180_deg, drivetrainMutex);
	RotationController turnTo175("turnTo175", drivetrain, odometry, turningPid, 175_deg, drivetrainMutex);
	RotationController turnTo225("turnTo225", drivetrain, odometry, turningPid, 225_deg, drivetrainMutex);
	RotationController turnTo270("turnTo270", drivetrain, odometry, turningPid, 270_deg, drivetrainMutex);
	RotationController turnTo315("TurnTo315", drivetrain, odometry, turningPid, 315_deg, drivetrainMutex);
	RotationController turnTo325("TurnTo325", drivetrain, odometry, turningPid, 325_deg, drivetrainMutex);
	RotationController turnTo305("TurnTo305", drivetrain, odometry, turningPid, 305_deg, drivetrainMutex);

	RamseteController testRamsete(&drivetrain, &odometry, intakeProfileConstraints, Pose2D(0_in, 24_in, 0_deg), 0.1, 0.0);

	void initDrivetrain() {
	}
} // namespace Pronounce
