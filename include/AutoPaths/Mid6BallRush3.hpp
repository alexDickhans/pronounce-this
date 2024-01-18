#pragma once
#include <vector>
#include "velocityProfile/sinusoidalVelocityProfile.hpp"
using namespace Pronounce;
std::vector<std::pair<PathPlanner::BezierSegment, Pronounce::SinusoidalVelocityProfile*>> MidRush3 = {{PathPlanner::BezierSegment(
PathPlanner::Point(31.4406_in, 13.8418_in),
PathPlanner::Point(36.5819_in, 32.2316_in),
PathPlanner::Point(38.9549_in, 41.7232_in),
PathPlanner::Point(48.8418_in, 55.565_in)
,false),
nullptr},
};
// PathPlanner made path
