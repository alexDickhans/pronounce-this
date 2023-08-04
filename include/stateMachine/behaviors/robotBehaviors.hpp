#pragma once

#include "pros/rtos.hpp"

pros::Mutex robotBehaviorMutex;

#include "drivetrain/joystick.hpp"
#include "drivetrain/initDrivetrain.hpp"

#include "intake/intake.hpp"
#include "intake/initIntake.hpp"

// TODO: Find a better solution for this
