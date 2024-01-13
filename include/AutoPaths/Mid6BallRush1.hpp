#pragma once
#include <vector>
#include "velocityProfile/sinusoidalVelocityProfile.hpp"
using namespace Pronounce;
std::vector<std::pair<PathPlanner::BezierSegment, Pronounce::SinusoidalVelocityProfile*>> MidRush1 = {{PathPlanner::BezierSegment(
PathPlanner::Point(69.209_in, 46.6667_in),
PathPlanner::Point(44.4915_in, 37.7684_in),
PathPlanner::Point(20.1695_in, 30.452_in),
PathPlanner::Point(12.8531_in, 44.2938_in)
,true),
nullptr},
};
// PathPlanner made path
