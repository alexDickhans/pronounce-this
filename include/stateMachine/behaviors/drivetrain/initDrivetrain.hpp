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

namespace Pronounce {

	PID turningPid(5.0, 0.0, 42.0, 0.0, 0.0, false);
	PID turningVisionPid(5.0, 0.0, 42.0, 0.0, 0.0, false);
	PID movingTurnPid(25000.0, 0.0, 100000.0, 0.0, 0.0, false);
	PID visionPid(0.0045, 0.0, 0.10);

	PID distancePid(400000.0, 0.0, 300000.0);

	// Drivetrain states for driving around the field and shooting at the goal
	JoystickDrivetrain normalJoystick("NormalJoystick", odometry, master, drivetrain, turningVisionPid, 0.10, false, 2.4, 73_in / second);
	JoystickDrivetrain targetingJoystick("TargetingJoystick", odometry, master, drivetrain, turningVisionPid, 0.10, true, 2.4, 73_in / second);
	Wait targetingJoystickStop(&targetingJoystick, 400_ms);

	JoystickDrivetrain drivetrainStopped("DrivetrainStopped", odometry, master, drivetrain, visionPid, 0.10, true, 2.4, 0.0);

	StateController drivetrainStateController("DrivetrainStateController", &drivetrainStopped);

	ProfileConstraints momentumProfileConstraints = { 70_in / second, 110_in / second / second, 0.0 };
	ProfileConstraints defaultProfileConstraints = { 70_in / second, 110_in / second / second, 0.0 };
	ProfileConstraints intakeBarrierProfileConstraints = { 30_in / second, 125_in / second / second, 0.0 };
	ProfileConstraints intakeProfileConstraints = { 50_in / second, 125_in / second / second, 0.0 };
	ProfileConstraints stackIntakeProfileConstraints = { 25_in / second, 130_in / second / second, 0.0 };

	RamseteController testRamsete(&drivetrain, &odometry, intakeProfileConstraints, Pose2D(0_in, 24_in, 0_deg), 0.0, 0.0);

	BezierPath testPath("TestPath");

	TankPurePursuit testPathPurePursuit("TestPurePursuit", &drivetrain, &odometry, { 5_in, SinusoidalVelocityProfile(10_in, intakeProfileConstraints) }, testPath);

	void initDrivetrain() {

		testPath.addPoint(SplinePoint(Point(0.0, 0.0), Vector(5_in, 0.0)));
		testPath.addPoint(SplinePoint(Point(24_in, 24_in), Vector(5_in, 0.0)));

		testPathPurePursuit.setPath(testPath.getPath(0.01));
	}
} // namespace Pronounce
