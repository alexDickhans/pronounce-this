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

	PID turningPid(2.0, 0.01, 22.0, 0.0, 0.0, false);
	PID movingTurnPid(2.4e4, 0.0, 2.64e5, 0.0, 0.0, true);

	PID distancePid(1.5e5, 0.0, 0e5);

	// Drivetrain states for driving around the field and shooting at the goal
	JoystickDrivetrain normalJoystick("NormalJoystick", odometry, master, drivetrain, 0.10, 2.4, 61_in / second);

	JoystickDrivetrain drivetrainStopped("DrivetrainStopped", odometry, master, drivetrain, 0.10, 2.4, 0.0);

	StateController drivetrainStateController("DrivetrainStateController", &drivetrainStopped);

	ProfileConstraints speedProfileConstraints = { 71_in / second, 280_in / second / second, 0.0 };
	ProfileConstraints defaultProfileConstraints = { 70_in / second, 140_in / second / second, 0.0 };
	ProfileConstraints pushingProfileConstraints = { 25_in / second, 70_in / second / second, 0.0 };

	PathPlanner::PathFollower pathFollower("PathFollower", defaultProfileConstraints, drivetrain, [ObjectPtr = &odometry] { return ObjectPtr->getAngle(); }, movingTurnPid, distancePid, 7000.0/72.0, 71_in/second, {});

	void initDrivetrain() {
		turningPid.setIntegralBound((20_deg).Convert(radian));
		turningPid.setMaxIntegral(25);
		movingTurnPid.setIntegralBound((20_deg).Convert(radian));
		movingTurnPid.setMaxIntegral(25);
	}
} // namespace Pronounce
