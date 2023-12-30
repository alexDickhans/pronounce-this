#pragma once
#include <vector>
#include "velocityProfile/sinusoidalVelocityProfile.hpp"
using namespace Pronounce;
std::vector<std::pair<PathPlanner::BezierSegment, Eigen::Vector3d>> Skills5 = {{PathPlanner::BezierSegment(
PathPlanner::Point(75.3391_in, 113.7_in),
PathPlanner::Point(81.6667_in, 77.1186_in),
PathPlanner::Point(130.31_in, 97.6834_in),
PathPlanner::Point(144.152_in, 88.1914_in)
,false),
{}},
{PathPlanner::BezierSegment(
PathPlanner::Point(137.429_in, 85.4233_in),
PathPlanner::Point(136.836_in, 115.875_in),
PathPlanner::Point(109.746_in, 133.474_in),
PathPlanner::Point(87.7967_in, 133.277_in)
,true),
{}},
};
// PathPlanner made path
