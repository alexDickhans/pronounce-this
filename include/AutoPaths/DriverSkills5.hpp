#pragma once
#include <vector>
#include "velocityProfile/sinusoidalVelocityProfile.hpp"
using namespace Pronounce;
std::vector<std::pair<PathPlanner::BezierSegment, Pronounce::SinusoidalVelocityProfile*>> DriverSkills5 = {{PathPlanner::BezierSegment(
PathPlanner::Point(75.5364_in, 116.864_in),
PathPlanner::Point(70.7909_in, 100.452_in),
PathPlanner::Point(56.1581_in, 106.582_in),
PathPlanner::Point(53.5876_in, 75.9322_in)
,false),
nullptr},
{PathPlanner::BezierSegment(
PathPlanner::Point(53.5876_in, 75.9322_in),
PathPlanner::Point(53.5875_in, 60.3107_in),
PathPlanner::Point(53.3899_in, 46.5677_in),
PathPlanner::Point(53.1921_in, 36.3841_in)
,false),
nullptr},
};
// PathPlanner made path
