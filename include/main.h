#ifndef _PROS_MAIN_H_
#define _PROS_MAIN_H_

//#define PROS_USE_LITERALS

#include "api.h"


/**
 * You should add more #includes here
 */
//#include "pros/api_legacy.h"

// 2654lib includes

#include "pathPlanner/pathPlanner.hpp"

#include "auton.h"

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
#include "odometry/continuousOdometry/particleFilterOdometry.hpp"

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
#include "utils/point.hpp"
#include "utils/pose2d.hpp"
#include "utils/runningAverage.hpp"
#include "utils/splinePoint.hpp"
#include "utils/utils.hpp"
#include "utils/vector.hpp"
#include "utils/polynomialExpression.hpp"

#include "velocityProfile/trapezoidalVelocityProfile.hpp"

#include "stateMachine/state/competition/auton.hpp"

#include "Logger/logger.hpp"

using namespace Pronounce;

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
//#include <iostream>
#endif

#endif  // _PROS_MAIN_H_
