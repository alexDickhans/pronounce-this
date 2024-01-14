#pragma once
#include <vector>
#include "velocityProfile/sinusoidalVelocityProfile.hpp"
using namespace Pronounce;
std::vector<std::pair<PathPlanner::BezierSegment, Pronounce::SinusoidalVelocityProfile*>> MidRush3 = {{PathPlanner::BezierSegment(
PathPlanner::Point(34.209_in, 13.8418_in),
PathPlanner::Point(35.9887_in, 38.1638_in),
PathPlanner::Point(45.2825_in, 53.3899_in),
PathPlanner::Point(46.2712_in, 66.0452_in)
,false),
nullptr},
};
// PathPlanner made path
