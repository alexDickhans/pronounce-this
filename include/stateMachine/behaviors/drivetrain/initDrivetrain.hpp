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
	PID movingTurnPid;//(2.0e4, 0.0, 4e4, 60000.0, 0.0, false);

	PID distancePid;//(1.5e5, 0.0, 0e5);

	// Drivetrain states for driving around the field and shooting at the goal
	JoystickDrivetrain normalJoystick("NormalJoystick", odometry, master, drivetrain, 0.10, 2.4, 61_in / second);

	JoystickDrivetrain drivetrainStopped("DrivetrainStopped", odometry, master, drivetrain, 0.10, 2.4, 0.0);

	StateController drivetrainStateController("DrivetrainStateController", &drivetrainStopped);

	Eigen::Vector3d defaultProfileConstraints = { (55_in / second).getValue(), (60_in / second / second).getValue(), (20_in / second / second).getValue() };
	ProfileConstraints oldDefaultProfileConstraints = { 61_in / second, 140_in / second / second, 0.0 };
	Eigen::Vector3d pushingProfileConstraints = { (25_in / second).getValue(), (100_in / second / second).getValue(), 0.0 };

	// Ks = 960 mv
	// kV = (4020-960)/19.9 in /sec
	// kA = (8000-1500)/187ins2

	PathPlanner::PathFollower pathFollower("PathFollower", defaultProfileConstraints, drivetrain, [ObjectPtr = &odometry] { return ObjectPtr->getAngle(); }, movingTurnPid, distancePid, [](QSpeed speed, QAcceleration acceleration) -> double {
		std::cout << "Acceleration: " << acceleration.Convert(inchs2) << std::endl;
		return 960.0 * signnum_c(speed.getValue()) + 153.76 * speed.Convert(inch/second) + 35.7*acceleration.Convert(inchs2);
		}, {});
} // namespace Pronounce
