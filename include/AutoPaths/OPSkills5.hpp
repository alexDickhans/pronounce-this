#pragma once
#include <vector>
#include "velocityProfile/sinusoidalVelocityProfile.hpp"
using namespace Pronounce;
std::vector<std::pair<PathPlanner::BezierSegment, Pronounce::SinusoidalVelocityProfile*>> OPSkills5 = {{PathPlanner::BezierSegment(
PathPlanner::Point(57.3446_in, 113.503_in),
PathPlanner::Point(52.4011_in, 94.7175_in),
PathPlanner::Point(11.2712_in, 122.006_in),
PathPlanner::Point(7.31638_in, 99.661_in)
,false),
nullptr},
{PathPlanner::BezierSegment(
PathPlanner::Point(7.31638_in, 99.661_in),
PathPlanner::Point(10.4802_in, 118.051_in),
PathPlanner::Point(22.048_in, 132.585_in),
PathPlanner::Point(42.3164_in, 132.486_in)
,true),
nullptr},
};
// PathPlanner made path
