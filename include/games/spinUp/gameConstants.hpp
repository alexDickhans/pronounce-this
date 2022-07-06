#pragma once

#include "utils/pointUtil.hpp"

namespace Pronounce {
	const class GameConstants {
	private:
		
	public:
		const Point ALLIANCE_GOAL = Point(120.0, 24.0);
		const Point OPPONENT_GOAL = Point(24.0, 120.0);
		
		const Point LEFT_ROLLER = Point(0, 72);
		const Point TOP_ROLLER = Point(24, 144);
		const Point RIGHT_ROLLER = Point(144, 120);
		const Point BOTTOM_ROLLER = Point(24, 0);

		GameConstants(/* args */);
		~GameConstants();
	};	
} // namespace Pronounce
