#pragma once
#include <vector>
#include "velocityProfile/sinusoidalVelocityProfile.hpp"
using namespace Pronounce;
std::vector<std::pair<PathPlanner::BezierSegment, Pronounce::SinusoidalVelocityProfile*>> Auto6BallElim6 = {{PathPlanner::BezierSegment(
PathPlanner::Point(42.5141_in, 12.6554_in),
PathPlanner::Point(9.29368_in, 23.9265_in),
PathPlanner::Point(30.6497_in, 55.9603_in),
PathPlanner::Point(27.2881_in, 73.3615_in)
,true),
nullptr},
};
// PathPlanner made path
