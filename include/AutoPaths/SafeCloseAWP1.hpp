#pragma once
#include <vector>
#include "velocityProfile/sinusoidalVelocityProfile.hpp"
using namespace Pronounce;
std::vector<std::pair<PathPlanner::BezierSegment, Pronounce::SinusoidalVelocityProfile*>> SafeCloseAWP1 = {{PathPlanner::BezierSegment(
PathPlanner::Point(121.808_in, 14.2377_in),
PathPlanner::Point(133.475_in, 23.7288_in),
PathPlanner::Point(121.016_in, 39.5476_in),
PathPlanner::Point(122.796_in, 52.2034_in)
,true),
nullptr},
};
// PathPlanner made path
