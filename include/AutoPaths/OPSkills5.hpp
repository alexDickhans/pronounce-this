#pragma once
#include <vector>
#include "velocityProfile/sinusoidalVelocityProfile.hpp"
using namespace Pronounce;
std::vector<std::pair<PathPlanner::BezierSegment, Pronounce::SinusoidalVelocityProfile*>> OPSkills5 = {{PathPlanner::BezierSegment(
PathPlanner::Point(61.1016_in, 113.503_in),
PathPlanner::Point(51.6101_in, 79.096_in),
PathPlanner::Point(13.0508_in, 133.871_in),
PathPlanner::Point(7.31638_in, 99.661_in)
,false),
new SinusoidalVelocityProfile(0.0, 61_in/second, 150_in/second/second, 0.0)},
{PathPlanner::BezierSegment(
PathPlanner::Point(7.31638_in, 99.661_in),
PathPlanner::Point(10.8757_in, 119.831_in),
PathPlanner::Point(22.048_in, 132.585_in),
PathPlanner::Point(42.3164_in, 132.486_in)
,true),
nullptr},
};
// PathPlanner made path
