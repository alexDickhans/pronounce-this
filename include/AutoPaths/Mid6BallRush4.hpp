#pragma once
#include <vector>
#include "velocityProfile/sinusoidalVelocityProfile.hpp"
using namespace Pronounce;
std::vector<std::pair<PathPlanner::BezierSegment, Pronounce::SinusoidalVelocityProfile*>> MidRush4 = {{PathPlanner::BezierSegment(
PathPlanner::Point(61.6949_in, 26.4972_in),
PathPlanner::Point(60.3108_in, 48.0509_in),
PathPlanner::Point(48.2486_in, 59.9152_in),
PathPlanner::Point(41.5254_in, 49.8305_in)
,true),
nullptr},
};
// PathPlanner made path
