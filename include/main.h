#ifndef _PROS_MAIN_H_
#define _PROS_MAIN_H_

//#define PROS_USE_LITERALS

#include "api.h"

#include "logger/logger.hpp"
/**
 * You should add more #includes here
 */
//#include "pros/api_legacy.h"

// 2654lib includes

#include "pathPlanner/pathPlanner.hpp"

#include "auton.h"

// Chassis
#include "chassis/tankDrive.hpp"

// FeedbackControllers
#include "feedbackControllers/pid.hpp"
#include "feedbackControllers/feedbackController.hpp"

// Motion control
#include "motionControl/rotationController.hpp"
#include "motionControl/tankMotionProfiling.hpp"

// Orientation
#include "odometry/orientation/imu.hpp"
#include "odometry/orientation/orientation.hpp"

#include "pathPlanner/pathPlanner.hpp"

// State Machine
#include "stateMachine/stateMachine.hpp"
#include "hardware/hardware.hpp"
#include "hardwareAbstractions/joystick/joystick.hpp"
#include "hardwareAbstractions/joystick/robotJoystick.hpp"

// Units
#include "units/units.hpp"

// Utils
#include "utils/point.hpp"
#include "utils/pose2d.hpp"
#include "utils/utils.hpp"

#include "velocityProfile/trapezoidalVelocityProfile.hpp"

#include "stateMachine/state/competition/auton.hpp"

#include "logger/logger.hpp"


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

using namespace Pronounce;

#ifdef __cplusplus
//#include <iostream>
#endif

#endif  // _PROS_MAIN_H_
