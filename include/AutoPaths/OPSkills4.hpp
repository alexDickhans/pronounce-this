#pragma once
#include <vector>
#include "velocityProfile/sinusoidalVelocityProfile.hpp"
using namespace Pronounce;
std::vector<std::pair<PathPlanner::BezierSegment, Pronounce::SinusoidalVelocityProfile*>> OPSkills4 = {{PathPlanner::BezierSegment(
PathPlanner::Point(77.9095_in, 112.91_in),
PathPlanner::Point(77.1185_in, 72.3729_in),
PathPlanner::Point(47.4577_in, 77.9096_in),
PathPlanner::Point(27.8814_in, 76.5254_in)
,false),
new SinusoidalVelocityProfile(0.0, 61_in/second, 150_in/second/second, 0.0)},
{PathPlanner::BezierSegment(
PathPlanner::Point(30.0565_in, 76.5254_in),
PathPlanner::Point(47.4576_in, 85.226_in),
PathPlanner::Point(56.257_in, 90.1695_in),
PathPlanner::Point(56.3559_in, 111.327_in)
,true),
nullptr},
};
// PathPlanner made path
