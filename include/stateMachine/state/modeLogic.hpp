#pragma once

#include "stateMachine/behaviors/robotBehaviors.hpp"
#include "stateMachine/behavior.hpp"
#include "stateMachine/behaviorGroup.hpp"
#include "stateMachine/parallel.hpp"
#include "utils/utils.hpp"

#include "competition/disabled.hpp"
#include "competition/enabled.hpp"
#include "competition/auton.hpp"
#include "competition/teleop.hpp"

#include "auton.h"

// TODO: Add docstrings

namespace Pronounce {
	Disabled disabled;
	Auton auton([](void *) -> void { printf("Auton"); });
	Teleop teleop(new Pronounce::RobotJoystick(static_cast<controller_id_e_t>(0)), new Pronounce::RobotJoystick(
			static_cast<controller_id_e_t>(1)));

	StateController competitionController("CompetitionController", &disabled);
} // namespace Pronounce
