#pragma once
#include <vector>
#include "velocityProfile/sinusoidalVelocityProfile.hpp"
using namespace Pronounce;
std::vector<std::pair<PathPlanner::BezierSegment, Pronounce::SinusoidalVelocityProfile*>> Skills5 = {{PathPlanner::BezierSegment(
PathPlanner::Point(75.3391_in, 113.7_in),
PathPlanner::Point(80.678_in, 89.3785_in),
PathPlanner::Point(123.39_in, 110.734_in),
PathPlanner::Point(137.232_in, 101.242_in)
,false),
nullptr},
{PathPlanner::BezierSegment(
PathPlanner::Point(135.847_in, 102.429_in),
PathPlanner::Point(135.254_in, 132.881_in),
PathPlanner::Point(109.746_in, 133.474_in),
PathPlanner::Point(87.7967_in, 133.277_in)
,true),
nullptr},
};
// PathPlanner made path
