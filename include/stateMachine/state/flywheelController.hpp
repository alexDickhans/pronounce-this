#pragma once

#include "games/spinUp/gameConstants.hpp"

// TODO: Finish implentation

namespace Pronounce {

	class FlywheelController :  {
	private:
		Point targetPosition;
	public:
		FlywheelController(Point targetPosition) {
			this->targetPosition = targetPosition;
		}


		
		~FlywheelController() {}
	};
} // namespace Pronounce
