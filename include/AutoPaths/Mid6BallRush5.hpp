#pragma once
#include <vector>
#include "velocityProfile/sinusoidalVelocityProfile.hpp"
using namespace Pronounce;
std::vector<std::pair<PathPlanner::BezierSegment, Pronounce::SinusoidalVelocityProfile*>> MidRush5 = {{PathPlanner::BezierSegment(
PathPlanner::Point(59.322_in, 27.0904_in),
PathPlanner::Point(59.1243_in, 42.7119_in),
PathPlanner::Point(59.9153_in, 57.1469_in),
PathPlanner::Point(68.6158_in, 65.6497_in)
,false),
nullptr},
};
// PathPlanner made path
