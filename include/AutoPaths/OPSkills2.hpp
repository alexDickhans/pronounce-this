#pragma once
#include <vector>
#include "velocityProfile/sinusoidalVelocityProfile.hpp"
using namespace Pronounce;
std::vector<std::pair<PathPlanner::BezierSegment, Pronounce::SinusoidalVelocityProfile*>> OPSkills2 = {{PathPlanner::BezierSegment(
PathPlanner::Point(93.1356_in, 81.4689_in),
PathPlanner::Point(103.418_in, 88.3898_in),
PathPlanner::Point(104.209_in, 95.7062_in),
PathPlanner::Point(93.3333_in, 101.441_in)
,true),
nullptr},
{PathPlanner::BezierSegment(
PathPlanner::Point(93.3333_in, 102.429_in),
PathPlanner::Point(84.0396_in, 102.627_in),
PathPlanner::Point(80.1836_in, 106.78_in),
PathPlanner::Point(79.887_in, 115.678_in)
,true),
nullptr},
};
// PathPlanner made path
