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

	constexpr double kS = 900; // mV
	constexpr double kV = 130; // mVsecond/inch
	constexpr double kA = 26.7; // mVs^2/inch

	double drivetrainFeedforward(QVelocity velocity, QAcceleration acceleration) {
		return signnum_c(velocity.getValue()) * kS + velocity.Convert(inch/second) * kV + acceleration.Convert(inch/second/second) * kA;
	}

	PID turningPid(2.0, 0.01, 22.0, 0.0, 0.0, false);
	PID movingTurnPid(2.4e4, 0.0, 2.64e5, 0.0, 0.0, true);

	PID distancePid(1.5e5, 0.0, 0e5);

	// Drivetrain states for driving around the field and shooting at the goal
	auto normalJoystick = std::make_shared<JoystickDrivetrain>("NormalJoystick", odometry, master, drivetrain, 0.01, 61_in / second);

	auto drivetrainStopped = std::make_shared<JoystickDrivetrain>("DrivetrainStopped", odometry, master, drivetrain, 0.02, 0.0);

	auto drivetrainStateController = std::make_shared<StateController>("DrivetrainStateController", drivetrainStopped);

	ProfileConstraints speedProfileConstraints = {76_in / second, 300_in / second / second, 0.0};
	ProfileConstraints defaultProfileConstraints = {70_in / second, 140_in / second / second, 0.0};
	ProfileConstraints pushingProfileConstraints = {50_in / second, 140_in / second / second, 0.0};

	auto pathFollower = std::make_shared<PathPlanner::PathFollower>(std::make_shared<PathPlanner::AbstractMotionProfile>(), drivetrain,
	                                                                movingTurnPid, distancePid, drivetrainFeedforward,
	                                                                [ObjectPtr = &odometry] { return ObjectPtr->getAngle(); });

	void initDrivetrain() {
		Log("Drivetrain Init");
		turningPid.setIntegralBound((20_deg).Convert(radian));
		turningPid.setMaxIntegral(25);
		movingTurnPid.setIntegralBound((20_deg).Convert(radian));
		movingTurnPid.setMaxIntegral(25);
		Log("Drivetrain Init Done");
	}
} // namespace Pronounce
