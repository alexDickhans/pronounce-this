#pragma once

#include "stateMachine/behaviors/robotBehaviors.hpp"
#include "stateMachine/behavior.hpp"
#include "stateMachine/behaviorGroup.hpp"
#include "utils/utils.hpp"

#include "competition/disabled.hpp"
#include "competition/enabled.hpp"
#include "competition/auton.hpp"
#include "competition/teleop.hpp"

#include "auton.h"

// TODO: Add docstrings

namespace Pronounce {
	auto disabled = std::make_shared<Disabled>();
	auto auton = std::make_shared<Auton>([](void *) -> void { printf("Auton"); });
	auto teleop = std::make_shared<Teleop>(master);

	auto competitionController = std::make_shared<StateController>("CompetitionController", disabled);
} // namespace Pronounce
