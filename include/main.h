/**
 * \file main.h
 *
 * Contains common definitions and header files used throughout your PROS
 * project.
 *
 * Copyright (c) 2017-2021, Purdue University ACM SIGBots.
 * All rights reserved.
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */

#ifndef _PROS_MAIN_H_
#define _PROS_MAIN_H_

/**
 * If defined, C++ literals will be available for use. All literals are in the
 * pros::literals namespace.
 *
 * For instance, you can do `4_mtr = 50` to set motor 4's target velocity to 50
 */
//#define PROS_USE_LITERALS

#include "api.h"


/**
 * You should add more #includes here
 */
//#include "pros/api_legacy.h"

// 2654lib includes

#include "pathPlanner/pathPlanner.hpp"

#include "defines.h"
#include "auton.h"
#include "driver.h"

// Chassis
#include "chassis/abstractDrivetrain.hpp"
#include "chassis/hardwareDrivetrain.hpp"
#include "chassis/tankDrive.hpp"

// FeedbackControllers
#include "feedbackControllers/bangBang.hpp"
#include "feedbackControllers/pid.hpp"
#include "feedbackControllers/feedbackController.hpp"
#include "feedbackControllers/flywheelPID.hpp"

// Motion control
#include "motionControl/rotationController.hpp"
#include "motionControl/tankMotionProfiling.hpp"

// Orientation
#include "odometry/orientation/avgOrientation.hpp"
#include "odometry/orientation/imu.hpp"
#include "odometry/orientation/orientation.hpp"

// Continuous Odometry
#include "odometry/continuousOdometry/continuousOdometry.hpp"
#include "odometry/continuousOdometry/threeWheelOdom.hpp"

// Interrupt Odom
#include "odometry/interruptOdometry/gpsOdometry.hpp"
#include "odometry/interruptOdometry/interruptOdometry.hpp"

#include "pathPlanner/pathPlanner.hpp"

// Position
#include "position/motorOdom.hpp"
#include "position/odomWheel.hpp"

// State Machine
#include "stateMachine/stateMachine.hpp"

#include "hardware/hardware.hpp"
#include "hardwareAbstractions/joystick/joystick.hpp"
#include "hardwareAbstractions/joystick/robotJoystick.hpp"

// Units
#include "units/units.hpp"

// Utils
#include "utils/ADIDigitalOutGroup.hpp"
#include "utils/exponentialMovingAverage.hpp"
#include "utils/point.hpp"
#include "utils/pose2d.hpp"
#include "utils/runningAverage.hpp"
#include "utils/splinePoint.hpp"
#include "utils/utils.hpp"
#include "utils/vector.hpp"
#include "utils/polynomialExpression.hpp"
#include "utils/path/combinedPath.hpp"

#include "telemetryRadio/telemetryManager.hpp"

#include "AutoPaths/6ball1.hpp"
#include "AutoPaths/6ball3.hpp"
#include "AutoPaths/Skills1.hpp"
#include "AutoPaths/Skills2.hpp"
#include "AutoPaths/Skills3.hpp"

/**
 * If you find doing pros::Motor() to be tedious and you'd prefer just to do
 * Motor, you can use the namespace with the following commented out line.
 *
 * IMPORTANT: Only the okapi or pros namespace may be used, not both
 * concurrently! The okapi namespace will export all symbols inside the pros
 * namespace.
 */
// using namespace pros;
// using namespace pros::literals;
using namespace Pronounce; // General Lib
//using namespace PathPlanner;

/**
 * Prototypes for the competition control tasks are redefined here to ensure
 * that they can be called from user code (i.e. calling autonomous from a
 * button press in opcontrol() for testing purposes).
 */
#ifdef __cplusplus
extern "C" {
#endif
void autonomous(void);
void initialize(void);
[[noreturn]] void disabled(void);
void competition_initialize(void);
void opcontrol(void);
#ifdef __cplusplus
}
#endif

#ifdef __cplusplus
/**
 * You can add C++-only headers here
 */
//#include <iostream>
#endif

#endif  // _PROS_MAIN_H_
