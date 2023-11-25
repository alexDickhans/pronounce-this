#pragma once
#include <vector>
#include "velocityProfile/sinusoidalVelocityProfile.hpp"
using namespace Pronounce;
std::vector<std::pair<PathPlanner::BezierSegment, Pronounce::SinusoidalVelocityProfile*>> OPSkills3 = {{PathPlanner::BezierSegment(
PathPlanner::Point(81.0734_in, 112.712_in),
PathPlanner::Point(81.0734_in, 91.1582_in),
PathPlanner::Point(81.0734_in, 82.6554_in),
PathPlanner::Point(89.5763_in, 76.9209_in)
,false),
nullptr},
{PathPlanner::BezierSegment(
PathPlanner::Point(89.5763_in, 76.9209_in),
PathPlanner::Point(77.5141_in, 84.6328_in),
PathPlanner::Point(71.3842_in, 88.6864_in),
PathPlanner::Point(71.1864_in, 112.119_in)
,true),
nullptr},
};
// PathPlanner made path
