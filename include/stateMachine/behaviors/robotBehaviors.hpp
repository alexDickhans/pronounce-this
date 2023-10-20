#pragma once

#include "pros/rtos.hpp"

pros::Mutex robotBehaviorMutex;

#include "drivetrain/joystick.hpp"
#include "drivetrain/initDrivetrain.hpp"

#include "intake/intake.hpp"
#include "intake/initIntake.hpp"

#include "catapult/catapult.hpp"
#include "catapult/catapultHold.hpp"
#include "catapult/initCatapult.hpp"

#include "wings/wings.hpp"
#include "wings/initWings.hpp"
