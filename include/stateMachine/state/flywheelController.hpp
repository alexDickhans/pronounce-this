#pragma once

#include "games/spinUp/gameConstants.hpp"

namespace Pronounce {
	class FlywheelController {
	private:
		Point currentGoal;
		bool skills = false;
	public:
		FlywheelController(Point currentGoal, bool skills) {
			this->currentGoal = currentGoal;
			this->skills = skills;
		}
		
		~FlywheelController() {}
	};
} // namespace Pronounce
