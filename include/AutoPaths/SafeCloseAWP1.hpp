#pragma once
#include <vector>
#include "velocityProfile/sinusoidalVelocityProfile.hpp"
using namespace Pronounce;
std::vector<std::pair<PathPlanner::BezierSegment, Eigen::Vector3d>> SafeCloseAWP1 = {{PathPlanner::BezierSegment(
PathPlanner::Point(121.808_in, 14.2377_in),
PathPlanner::Point(130.113_in, 22.1469_in),
PathPlanner::Point(129.124_in, 38.3612_in),
PathPlanner::Point(129.52_in, 52.0057_in)
,true),
{}},
};
// PathPlanner made path
