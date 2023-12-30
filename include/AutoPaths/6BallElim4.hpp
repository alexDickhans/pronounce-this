#pragma once
#include <vector>
#include "velocityProfile/sinusoidalVelocityProfile.hpp"
using namespace Pronounce;
std::vector<std::pair<PathPlanner::BezierSegment, Eigen::Vector3d>> Auto6BallElim4 = {{PathPlanner::BezierSegment(
PathPlanner::Point(10.4802_in, 44.2938_in),
PathPlanner::Point(10.2825_in, 52.2034_in),
PathPlanner::Point(9.88701_in, 56.9492_in),
PathPlanner::Point(10.0847_in, 64.8588_in)
,false),
{}},
{PathPlanner::BezierSegment(
PathPlanner::Point(10.0847_in, 64.8588_in),
PathPlanner::Point(10.4802_in, 33.2203_in),
PathPlanner::Point(6.03107_in, 5.33898_in),
PathPlanner::Point(43.1073_in, 5.14124_in)
,true),
{}},
};
// PathPlanner made path
