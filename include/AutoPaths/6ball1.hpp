#pragma once
#include <vector>
#include "velocityProfile/sinusoidalVelocityProfile.hpp"
using namespace Pronounce;
std::vector<std::pair<PathPlanner::BezierSegment, QSpeed>> Auto6BallPath1 = {{PathPlanner::BezierSegment(
PathPlanner::Point(132_in, 77_in),
PathPlanner::Point(132_in, 76_in),
PathPlanner::Point(132_in, 76_in),
PathPlanner::Point(132_in, 74_in)
,false),
0.0},
{PathPlanner::BezierSegment(
PathPlanner::Point(132_in, 74_in),
PathPlanner::Point(132_in, 115_in),
PathPlanner::Point(120_in, 124.118_in),
PathPlanner::Point(95_in, 122.119_in)
,true),
0.0},
{PathPlanner::BezierSegment(
PathPlanner::Point(100_in, 124_in),
PathPlanner::Point(103.616_in, 124_in),
PathPlanner::Point(107.373_in, 124.181_in),
PathPlanner::Point(110.989_in, 124.373_in)
,false),
0.0},
};
// PathPlanner made path
