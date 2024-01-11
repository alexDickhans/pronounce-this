#pragma once
#include <vector>
#include "velocityProfile/sinusoidalVelocityProfile.hpp"
using namespace Pronounce;
std::vector<std::pair<PathPlanner::BezierSegment, Pronounce::SinusoidalVelocityProfile*>> Auto6BallElim2 = {{PathPlanner::BezierSegment(
PathPlanner::Point(69.4068_in, 25.5085_in),
PathPlanner::Point(66.6384_in, 35.5932_in),
PathPlanner::Point(63.2769_in, 39.5481_in),
PathPlanner::Point(54.3786_in, 40.1413_in)
,true),
nullptr},
};
// PathPlanner made path
