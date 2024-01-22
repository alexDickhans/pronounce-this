#pragma once
#include <vector>
#include "velocityProfile/sinusoidalVelocityProfile.hpp"
using namespace Pronounce;
std::vector<std::pair<PathPlanner::BezierSegment, Pronounce::SinusoidalVelocityProfile*>> DriverSkills5 = {{PathPlanner::BezierSegment(
PathPlanner::Point(75.9319_in, 116.666_in),
PathPlanner::Point(72.5706_in, 100.65_in),
PathPlanner::Point(56.1581_in, 106.582_in),
PathPlanner::Point(53.5876_in, 75.9322_in)
,false),
nullptr},
{PathPlanner::BezierSegment(
PathPlanner::Point(53.5876_in, 75.9322_in),
PathPlanner::Point(53.5875_in, 60.3107_in),
PathPlanner::Point(53.5876_in, 49.3361_in),
PathPlanner::Point(53.3898_in, 42.1186_in)
,false),
nullptr},
};
// PathPlanner made path
