#pragma once
#include <vector>
#include "velocityProfile/sinusoidalVelocityProfile.hpp"
using namespace Pronounce;
std::vector<std::pair<PathPlanner::BezierSegment, Eigen::Vector3d>> Auto6BallElim1 = {{PathPlanner::BezierSegment(
PathPlanner::Point(18.9831_in, 39.7458_in),
PathPlanner::Point(35.791_in, 43.8983_in),
PathPlanner::Point(51.0169_in, 61.4972_in),
PathPlanner::Point(68.8136_in, 62.4859_in)
,false),
{}},
};
// PathPlanner made path
