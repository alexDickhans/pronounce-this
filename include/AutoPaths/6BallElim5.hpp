#pragma once
#include <vector>
#include "velocityProfile/sinusoidalVelocityProfile.hpp"
using namespace Pronounce;
std::vector<std::pair<PathPlanner::BezierSegment, Eigen::Vector3d>> Auto6BallElim5 = {{PathPlanner::BezierSegment(
PathPlanner::Point(31.2429_in, 10.2825_in),
PathPlanner::Point(31.2429_in, 24.5198_in),
PathPlanner::Point(34.6045_in, 46.0734_in),
PathPlanner::Point(34.6045_in, 63.2768_in)
,false),
{}},
};
// PathPlanner made path
