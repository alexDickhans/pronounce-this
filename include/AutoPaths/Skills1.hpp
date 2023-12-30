#pragma once
#include <vector>
#include "velocityProfile/sinusoidalVelocityProfile.hpp"
using namespace Pronounce;
std::vector<std::pair<PathPlanner::BezierSegment, Eigen::Vector3d>> Skills1 = {{PathPlanner::BezierSegment(
PathPlanner::Point(16.4124_in, 30.2542_in),
PathPlanner::Point(25.7062_in, 29.661_in),
PathPlanner::Point(22.7402_in, 17.4011_in),
PathPlanner::Point(19.1808_in, 5.9322_in)
,true),
{}},
};
// PathPlanner made path
