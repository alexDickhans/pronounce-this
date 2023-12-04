#pragma once
#include <vector>
#include "velocityProfile/sinusoidalVelocityProfile.hpp"
using namespace Pronounce;
std::vector<std::pair<PathPlanner::BezierSegment, Pronounce::SinusoidalVelocityProfile*>> Auto6BallPath4 = {{PathPlanner::BezierSegment(
PathPlanner::Point(70_in, 80_in),
PathPlanner::Point(70_in, 90_in),
PathPlanner::Point(70_in, 100_in),
PathPlanner::Point(70_in, 112_in)
,false),
nullptr},
{PathPlanner::BezierSegment(
PathPlanner::Point(70_in, 115_in),
PathPlanner::Point(70_in, 85_in),
PathPlanner::Point(94.8305_in, 72.3333_in),
PathPlanner::Point(115.311_in, 77.8136_in)
,true),
nullptr},
};
// PathPlanner made path
