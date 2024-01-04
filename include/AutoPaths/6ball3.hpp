#pragma once
#include <vector>
#include "velocityProfile/sinusoidalVelocityProfile.hpp"
using namespace Pronounce;
std::vector<std::pair<PathPlanner::BezierSegment, QSpeed>> Auto6BallPath3 = {{PathPlanner::BezierSegment(
PathPlanner::Point(93_in, 90_in),
PathPlanner::Point(89.2429_in, 98.4181_in),
PathPlanner::Point(80.4802_in, 100.452_in),
PathPlanner::Point(78.1073_in, 114.294_in)
,false),
0.0},
{PathPlanner::BezierSegment(
PathPlanner::Point(81.6667_in, 115.48_in),
PathPlanner::Point(84.435_in, 106.384_in),
PathPlanner::Point(87.4017_in, 96.6947_in),
PathPlanner::Point(97.0894_in, 94.7175_in)
,true),
0.0},
{PathPlanner::BezierSegment(
PathPlanner::Point(106.187_in, 88.7854_in),
PathPlanner::Point(96.6945_in, 88.3896_in),
PathPlanner::Point(94.9153_in, 85.4235_in),
PathPlanner::Point(94.9152_in, 79.4914_in)
,false),
0.0},
};
// PathPlanner made path
