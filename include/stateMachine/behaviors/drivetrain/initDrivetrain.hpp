#pragma once

#include "joystick.hpp"
#include "api.h"
#include "utils/motorGroup.hpp"
#include "stateMachine/wait.hpp"
#include "stateMachine/stateController.hpp"
#include "stateMachine/behavior.hpp"
#include "odometry/continuousOdometry/continuousOdometry.hpp"
#include "position/trackingWheel.hpp"
#include "chassis/xdrive.hpp"
#include "driver/controller.hpp"
#include "utils/runningAverage.hpp"
#include "odometry/odomFuser.hpp"
#include "motionControl/omniMotionProfiling.hpp"
#include "velocityProfile/sinusoidalVelocityProfile.hpp"
#include "utils/path.hpp"
#include "utils/quadraticSplinePath.hpp"
#include "motionControl/rotationController.hpp"
#include "motionControl/omniMovement.hpp"
#include "hardware/hardware.hpp"

// TODO: Clean up
// TODO: move declarations to another place
// TODO: Add comments

namespace Pronounce {
	
	JoystickDrivetrain normalJoystick("RobotRelativeJoystick", 0.10, false, false, 2.4, 200.0, &movingAverageX, &movingAverageY, &movingAverageTurn, &odometry, &master, &drivetrain);
	JoystickDrivetrain normalTargetingJoystick("RobotRelativeTargeting", 0.10, false, true, 2.4, 200.0, &movingAverageX, &movingAverageY, &movingAverageTurn, &odometry, &master, &drivetrain);

	JoystickDrivetrain drivetrainStopped("DrivetrainStopped", 0.10, false, true, 2.4, 0.0, &movingAverageX, &movingAverageY, &movingAverageTurn, &odometry, &master, &drivetrain);

	PID turningPid(330, 0, 2000);

	StateController drivetrainStateController("DrivetrainStateController", &drivetrainStopped);

	void initDrivetrain() {

	}
} // namespace Pronounce
