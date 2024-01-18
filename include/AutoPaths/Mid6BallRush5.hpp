#pragma once
#include <vector>
#include "velocityProfile/sinusoidalVelocityProfile.hpp"
using namespace Pronounce;
std::vector<std::pair<PathPlanner::BezierSegment, Pronounce::SinusoidalVelocityProfile*>> MidRush5 = {{PathPlanner::BezierSegment(
PathPlanner::Point(60.5085_in, 45.2825_in),
PathPlanner::Point(60.7063_in, 55.1695_in),
PathPlanner::Point(63.0791_in, 59.9153_in),
PathPlanner::Point(68.6158_in, 65.6497_in)
,false),
nullptr},
};
// PathPlanner made path
