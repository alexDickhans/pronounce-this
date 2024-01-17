#pragma once
#include <vector>
#include "velocityProfile/sinusoidalVelocityProfile.hpp"
using namespace Pronounce;
std::vector<std::pair<PathPlanner::BezierSegment, Pronounce::SinusoidalVelocityProfile*>> MidRush1 = {{PathPlanner::BezierSegment(
PathPlanner::Point(69.209_in, 46.6667_in),
PathPlanner::Point(53.5876_in, 42.7119_in),
PathPlanner::Point(32.0339_in, 28.0791_in),
PathPlanner::Point(19.1808_in, 46.0735_in)
,true),
nullptr},
};
// PathPlanner made path
