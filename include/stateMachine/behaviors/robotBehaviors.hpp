#pragma once

#include "pros/rtos.hpp"

pros::Mutex robotBehaviorMutex;

#include "drivetrain/joystick.hpp"
#include "drivetrain/initDrivetrain.hpp"

#include "pto/intake.hpp"
#include "pto/initPto.hpp"
#include "pto/catapult.hpp"

#include "endgame/endgame.hpp"
#include "endgame/initEndgame.hpp"

#include "pistonBoost/pistonBoost.hpp"
#include "pistonBoost/initBoost.hpp"

// TODO: Find a better solution for this
