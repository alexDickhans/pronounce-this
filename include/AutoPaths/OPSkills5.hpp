#pragma once
#include <vector>
#include "velocityProfile/sinusoidalVelocityProfile.hpp"
using namespace Pronounce;
std::vector<std::pair<PathPlanner::BezierSegment, Pronounce::SinusoidalVelocityProfile*>> OPSkills5 = {{PathPlanner::BezierSegment(
PathPlanner::Point(61.1016_in, 113.503_in),
PathPlanner::Point(52.4011_in, 82.2598_in),
PathPlanner::Point(30.0565_in, 79.8879_in),
PathPlanner::Point(-18.7853_in, 79.4915_in)
,false),
new SinusoidalVelocityProfile(0.0, 61_in/second, 150_in/second/second, 0.0)},
{PathPlanner::BezierSegment(
PathPlanner::Point(7.71186_in, 75.1412_in),
PathPlanner::Point(7.9096_in, 112.317_in),
PathPlanner::Point(19.4774_in, 132.585_in),
PathPlanner::Point(42.3164_in, 132.486_in)
,true),
nullptr},
};
// PathPlanner made path
