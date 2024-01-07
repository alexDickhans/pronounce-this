#pragma once
#include <vector>
#include "velocityProfile/sinusoidalVelocityProfile.hpp"
using namespace Pronounce;
std::vector<std::pair<PathPlanner::BezierSegment, Pronounce::SinusoidalVelocityProfile*>> Auto6BallElim6 = {{PathPlanner::BezierSegment(
PathPlanner::Point(42.5141_in, 12.6554_in),
PathPlanner::Point(15.8191_in, 22.3446_in),
PathPlanner::Point(32.6271_in, 55.9603_in),
PathPlanner::Point(29.2655_in, 73.3615_in)
,true),
nullptr},
};
// PathPlanner made path
