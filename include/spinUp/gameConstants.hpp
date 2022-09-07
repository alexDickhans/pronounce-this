#pragma once

#include "utils/pointUtil.hpp"
#include "utils/path.hpp"

namespace Pronounce {
	const Point ALLIANCE_GOAL = Point(18.0_in, 122.5_in);
	const Point OPPONENT_GOAL = Point(122.5_in, 18.0_in);

	const Point bottomRoller = Point(29.5_in, 4_in);
	const Point leftRoller = Point(29.5_in, 4_in);
	const Point rightRoller = Point(140_in, 110.0_in);
	const Point topRoller = Point(110_in, 140_in);

	const Point allianceMatchloader = Point(70_in, 0_in);
	const Point opponentMatchloader = Point(70_in, 140_in);

	const Point singleDiscs[] = {Point(11.3_in, 11.3_in), Point(23_in, 23_in), Point(46.5_in, 46.5_in), Point(58.5_in, 58.5_in), Point(82.0_in, 82.0_in), Point(94.0_in, 94.0_in), Point(117.5_in, 117.5_in), Point(129.0_in, 129.0_in),
								 Point(82.0_in, 58.5_in), Point(94.0_in, 70.0_in), Point(105.5_in, 82.0_in),
								 Point(35.0_in, 58.5_in), Point(46.5_in, 70.0_in), Point(58.5_in, 82.0_in),
								 Point(26.3_in, 90.0_in), Point(35.0_in, 90.0_in), Point(44.5_in, 90.0_in),
								 Point(50.5_in, 96.5_in), Point(50.5_in, 105.5_in), Point(50.5_in, 114.0_in),
								 Point(90.0_in, 26.0_in), Point(90.5_in, 35.0_in), Point(90.0_in, 44.0_in),
								 Point(96.5_in, 50.5_in), Point(105.5_in, 50.5_in), Point(114.0_in, 50.5_in)};
	
	const Point tripleDiscs[] = {Point(35_in, 35_in), Point(58.5_in, 35_in), Point(82_in, 105.5_in), Point(105.5_in, 105.5_in)};

} // namespace Pronounce
