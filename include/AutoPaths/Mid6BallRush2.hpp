#pragma once
#include <vector>
#include "velocityProfile/sinusoidalVelocityProfile.hpp"
using namespace Pronounce;
std::vector<std::pair<PathPlanner::BezierSegment, Pronounce::SinusoidalVelocityProfile*>> MidRush2 = {{PathPlanner::BezierSegment(
PathPlanner::Point(11.2712_in, 69.6045_in),
PathPlanner::Point(11.2712_in, 10.2825_in),
PathPlanner::Point(29.2655_in, 9.09605_in),
PathPlanner::Point(44.6893_in, 8.70057_in)
,true),
nullptr},
};
// PathPlanner made path
