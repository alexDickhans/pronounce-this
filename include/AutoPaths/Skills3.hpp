#pragma once
#include <vector>
#include "velocityProfile/sinusoidalVelocityProfile.hpp"
using namespace Pronounce;
std::vector<std::pair<PathPlanner::BezierSegment, QSpeed>> Skills3 = {{PathPlanner::BezierSegment(
PathPlanner::Point(46.0734_in, 93.1355_in),
PathPlanner::Point(53.5876_in, 104.605_in),
PathPlanner::Point(55.9605_in, 111.722_in),
PathPlanner::Point(57.5423_in, 126.158_in)
,true),
0.0},
{PathPlanner::BezierSegment(
PathPlanner::Point(57.1469_in, 108.757_in),
PathPlanner::Point(48.644_in, 82.0622_in),
PathPlanner::Point(59.7174_in, 79.294_in),
PathPlanner::Point(80.6779_in, 78.5028_in)
,false),
0.0},
};
// PathPlanner made path
