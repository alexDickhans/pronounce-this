#pragma once
#include <vector>
#include "velocityProfile/sinusoidalVelocityProfile.hpp"
using namespace Pronounce;
std::vector<std::pair<PathPlanner::BezierSegment, Pronounce::SinusoidalVelocityProfile*>> OPSkills7 = {{PathPlanner::BezierSegment(
PathPlanner::Point(97.0904_in, 128.927_in),
PathPlanner::Point(122.797_in, 127.74_in),
PathPlanner::Point(114.491_in, 116.469_in),
PathPlanner::Point(99.4631_in, 105.593_in)
,false),
nullptr},
{PathPlanner::BezierSegment(
PathPlanner::Point(99.0674_in, 105_in),
PathPlanner::Point(83.6441_in, 92.1469_in),
PathPlanner::Point(69.4068_in, 94.9154_in),
PathPlanner::Point(69.6046_in, 85.0283_in)
,false),
nullptr},
{PathPlanner::BezierSegment(
PathPlanner::Point(69.6046_in, 83.6441_in),
PathPlanner::Point(70_in, 91.1582_in),
PathPlanner::Point(70.1977_in, 107.373_in),
PathPlanner::Point(70_in, 115.085_in)
,true),
nullptr},
};
// PathPlanner made path
