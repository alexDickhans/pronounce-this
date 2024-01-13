#pragma once
#include <vector>
#include "velocityProfile/sinusoidalVelocityProfile.hpp"
using namespace Pronounce;
std::vector<std::pair<PathPlanner::BezierSegment, Pronounce::SinusoidalVelocityProfile*>> MidRush3 = {{PathPlanner::BezierSegment(
PathPlanner::Point(43.5028_in, 14.0395_in),
PathPlanner::Point(11.6667_in, 22.7401_in),
PathPlanner::Point(53.1921_in, 54.9718_in),
PathPlanner::Point(69.0113_in, 65.6497_in)
,false),
nullptr},
{PathPlanner::BezierSegment(
PathPlanner::Point(69.0113_in, 65.6497_in),
PathPlanner::Point(63.2768_in, 54.774_in),
PathPlanner::Point(57.5424_in, 52.1045_in),
PathPlanner::Point(56.7514_in, 42.5141_in)
,true),
nullptr},
};
// PathPlanner made path
