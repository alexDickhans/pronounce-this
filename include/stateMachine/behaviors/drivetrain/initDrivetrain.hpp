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
#include "motionControl/rotationController.hpp"
#include "hardware/hardware.hpp"
#include "motionControl/tankMotionProfiling.hpp"
#include "chassis/tankDrive.hpp"
#include "motionControl/ramsete.hpp"

namespace Pronounce {

	PID turningPid(3.0, 0.0, 30.0, 0.0, 0.0, false);
	PID movingTurnPid(15000.0, 0.0, 100000.0, 0.0, 0.0, false);

	PID distancePid(400000.0, 0.0, 250000.0);

	// Drivetrain states for driving around the field and shooting at the goal
	JoystickDrivetrain normalJoystick("NormalJoystick", odometry, master, drivetrain, 0.10, 2.4, 61_in / second);

	JoystickDrivetrain drivetrainStopped("DrivetrainStopped", odometry, master, drivetrain, 0.10, 2.4, 0.0);

	StateController drivetrainStateController("DrivetrainStateController", &drivetrainStopped);

	ProfileConstraints defaultProfileConstraints = { 40_in / second, 100_in / second / second, 0.0 };

	TankMotionProfiling* getMPInstance(const CombinedPath& path, ProfileConstraints profileConstraints, Angle startAngle, QSpeed initialSpeed = 0.0, QSpeed finalSpeed = 0.0) {
		return new TankMotionProfiling(
				"MPInstance",
				&drivetrain,
				profileConstraints,
				path,
				&odometry,
				&distancePid,
				drivetrainMutex,
				startAngle,
				&turningPid,
				initialSpeed,
				finalSpeed);
	}

	void initDrivetrain() {

	}
} // namespace Pronounce
