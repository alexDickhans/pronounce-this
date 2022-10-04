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
 * If defined, some commonly used enums will have preprocessor macros which give
 * a shorter, more convenient naming pattern. If this isn't desired, simply
 * comment the following line out.
 *
 * For instance, E_CONTROLLER_MASTER has a shorter name: CONTROLLER_MASTER.
 * E_CONTROLLER_MASTER is pedantically correct within the PROS styleguide, but
 * not convienent for most student programmers.
 */
#define PROS_USE_SIMPLE_NAMES

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


#include "defines.h"
#include "auton.h"
#include "driver.h"

// Chassis
#include "chassis/abstractDrivetrain.hpp"
#include "chassis/drivetrain.hpp"
#include "chassis/abstractHolonomicDrivetrain.hpp"
#include "chassis/tankDrive.hpp"
#include "chassis/xdrive.hpp"

// Driver
#include "driver/controller.hpp"

// FeedbackControllers
#include "feedbackControllers/bangBang.hpp"
#include "feedbackControllers/pid.hpp"
#include "feedbackControllers/feedbackController.hpp"
#include "feedbackControllers/flywheelPID.hpp"

// Motion control
#include "motionControl/purePursuit.hpp"
#include "motionControl/omniPurePursuit.hpp"
#include "motionControl/tankPurePursuit.hpp"

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
#include "odometry/interruptOdometry/tapeOdometry.hpp"

// Position
#include "position/motorOdom.hpp"
#include "position/odomWheel.hpp"
#include "position/trackingWheel.hpp"

// State Machine
#include "stateMachine/behavior.hpp"
#include "stateMachine/sequence.hpp"
#include "stateMachine/stateController.hpp"
#include "stateMachine/wait.hpp"
#include "stateMachine/behaviors/robotBehaviors.hpp"
#include "stateMachine/state/modeLogic.hpp"
#include "stateMachine/state/teleopModeLogic.hpp"
#include "stateMachine/state/modeLogic.hpp"
#include "hardware/hardware.hpp"

// Units
#include "units/units.hpp"

// Utils
#include "utils/ADIDigitalOutGroup.hpp"
#include "utils/exponentialMovingAverage.hpp"
#include "utils/motorGroup.hpp"
#include "utils/path.hpp"
#include "utils/pointUtil.hpp"
#include "utils/pose2d.hpp"
#include "utils/quadraticSplinePath.hpp"
#include "utils/runningAverage.hpp"
#include "utils/splinePath.hpp"
#include "utils/splinePoint.hpp"
#include "utils/utils.hpp"
#include "utils/vector.hpp"
#include "utils/motorGroup.hpp"

// Vision


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
void disabled(void);
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
