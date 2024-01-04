#pragma once
#include <vector>
#include "velocityProfile/sinusoidalVelocityProfile.hpp"
using namespace Pronounce;
std::vector<std::pair<PathPlanner::BezierSegment, QSpeed>> Auto6BallPath4 = {{PathPlanner::BezierSegment(
PathPlanner::Point(70_in, 80_in),
PathPlanner::Point(70_in, 90_in),
PathPlanner::Point(70_in, 100_in),
PathPlanner::Point(70_in, 112_in)
,false),
0.0},
{PathPlanner::BezierSegment(
PathPlanner::Point(70_in, 115_in),
PathPlanner::Point(70_in, 85_in),
PathPlanner::Point(89.6892_in, 77.0791_in),
PathPlanner::Point(115.311_in, 77.0227_in)
,true),
0.0},
};
// PathPlanner made path
