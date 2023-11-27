#pragma once
#include <vector>
#include "velocityProfile/sinusoidalVelocityProfile.hpp"
using namespace Pronounce;
std::vector<std::pair<PathPlanner::BezierSegment, Pronounce::SinusoidalVelocityProfile*>> OPSkills2 = {{PathPlanner::BezierSegment(
PathPlanner::Point(69.4068_in, 76.5254_in),
PathPlanner::Point(69.2087_in, 87.2034_in),
PathPlanner::Point(69.6045_in, 105.198_in),
PathPlanner::Point(69.6045_in, 115.678_in)
,true),
nullptr},
};
// PathPlanner made path
