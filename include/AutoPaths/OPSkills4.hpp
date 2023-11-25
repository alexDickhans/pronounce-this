#pragma once
#include <vector>
#include "velocityProfile/sinusoidalVelocityProfile.hpp"
using namespace Pronounce;
std::vector<std::pair<PathPlanner::BezierSegment, Pronounce::SinusoidalVelocityProfile*>> OPSkills4 = {{PathPlanner::BezierSegment(
PathPlanner::Point(69.209_in, 112.712_in),
PathPlanner::Point(68.8136_in, 83.8418_in),
PathPlanner::Point(48.2486_in, 80.8757_in),
PathPlanner::Point(30.0565_in, 76.5254_in)
,false),
nullptr},
{PathPlanner::BezierSegment(
PathPlanner::Point(30.0565_in, 76.5254_in),
PathPlanner::Point(47.4576_in, 85.6215_in),
PathPlanner::Point(59.2232_in, 90.9604_in),
PathPlanner::Point(59.1243_in, 111.921_in)
,true),
nullptr},
};
// PathPlanner made path
