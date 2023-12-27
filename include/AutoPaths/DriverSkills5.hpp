#pragma once
#include <vector>
#include "velocityProfile/sinusoidalVelocityProfile.hpp"
using namespace Pronounce;
std::vector<std::pair<PathPlanner::BezierSegment, Pronounce::SinusoidalVelocityProfile*>> DriverSkills5 = {{PathPlanner::BezierSegment(
PathPlanner::Point(84.8305_in, 122.797_in),
PathPlanner::Point(80.8757_in, 102.825_in),
PathPlanner::Point(58.3333_in, 96.8927_in),
PathPlanner::Point(53.5876_in, 75.9322_in)
,false),
nullptr},
{PathPlanner::BezierSegment(
PathPlanner::Point(53.5876_in, 75.9322_in),
PathPlanner::Point(50.8192_in, 62.6836_in),
PathPlanner::Point(49.2373_in, 54.0819_in),
PathPlanner::Point(48.2486_in, 46.4689_in)
,false),
nullptr},
};
// PathPlanner made path
