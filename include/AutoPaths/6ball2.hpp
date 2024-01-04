#pragma once
#include <vector>
#include "velocityProfile/sinusoidalVelocityProfile.hpp"
using namespace Pronounce;
std::vector<std::pair<PathPlanner::BezierSegment, QSpeed>> Auto6BallPath2 = {{PathPlanner::BezierSegment(
PathPlanner::Point(108.593_in, 127.803_in),
PathPlanner::Point(98.6159_in, 108.504_in),
PathPlanner::Point(93.5367_in, 99.209_in),
PathPlanner::Point(93.1412_in, 81.7232_in)
,false),
0.0},
{PathPlanner::BezierSegment(
PathPlanner::Point(93_in, 70_in),
PathPlanner::Point(93_in, 80_in),
PathPlanner::Point(90_in, 80_in),
PathPlanner::Point(90_in, 87_in)
,true),
0.0},
};
// PathPlanner made path
