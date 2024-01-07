#pragma once

#include "joystick.hpp"
#include "api.h"
#include "stateMachine/wait.hpp"
#include "stateMachine/stateController.hpp"
#include "stateMachine/behavior.hpp"
#include "odometry/continuousOdometry/continuousOdometry.hpp"
#include "utils/runningAverage.hpp"
#include "odometry/odomFuser.hpp"
#include "velocityProfile/sinusoidalVelocityProfile.hpp"
#include "motionControl/rotationController.hpp"
#include "hardware/hardware.hpp"
#include "motionControl/tankMotionProfiling.hpp"
#include "chassis/tankDrive.hpp"

namespace Pronounce {

	PID turningPid(2.0, 0.0, 25.0, 0.0, 0.0, false);
	PID movingTurnPid(2.0e4, 0.0, 4e4, 60000.0, 0.0, false);

	PID distancePid(1.5e5, 0.0, 0e5);

	// Drivetrain states for driving around the field and shooting at the goal
	JoystickDrivetrain normalJoystick("NormalJoystick", odometry, master, drivetrain, 0.10, 2.4, 61_in / second);

	JoystickDrivetrain drivetrainStopped("DrivetrainStopped", odometry, master, drivetrain, 0.10, 2.4, 0.0);

	StateController drivetrainStateController("DrivetrainStateController", &drivetrainStopped);

	ProfileConstraints defaultProfileConstraints = { 61_in / second, 140_in / second / second, 0.0 };
	ProfileConstraints pushingProfileConstraints = { 25_in / second, 100_in / second / second, 0.0 };

	PathPlanner::PathFollower pathFollower("PathFollower", defaultProfileConstraints, drivetrain, [ObjectPtr = &odometry] { return ObjectPtr->getAngle(); }, movingTurnPid, distancePid, 8000.0/64.0, 61_in/second, {});

	void initDrivetrain() {

	}
} // namespace Pronounce
