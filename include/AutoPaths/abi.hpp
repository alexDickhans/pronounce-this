#pragma once
#include <vector>
#include "velocityProfile/sinusoidalVelocityProfile.hpp"
using namespace Pronounce;
std::vector<std::pair<PathPlanner::BezierSegment, Pronounce::SinusoidalVelocityProfile*>> ChangeMe = {{PathPlanner::BezierSegment(
PathPlanner::Point(11.2712_in, 70.1977_in),
PathPlanner::Point(11.2712_in, 10.8757_in),
PathPlanner::Point(21.5536_in, 17.2034_in),
PathPlanner::Point(41.525_in, 12.6554_in)
,false),
nullptr},
{PathPlanner::BezierSegment(
PathPlanner::Point(41.525_in, 12.6554_in),
PathPlanner::Point(61.4964_in, 8.10738_in),
PathPlanner::Point(80.2821_in, 41.822_in),
PathPlanner::Point(99.0678_in, 75.5367_in)
,false),
nullptr},
};
// PathPlanner made path
