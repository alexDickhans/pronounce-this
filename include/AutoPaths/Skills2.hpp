#pragma once
#include <vector>
#include "velocityProfile/sinusoidalVelocityProfile.hpp"
using namespace Pronounce;
std::vector<std::pair<PathPlanner::BezierSegment, Pronounce::SinusoidalVelocityProfile*>> Skills3 = {{PathPlanner::BezierSegment(
PathPlanner::Point(74.9435_in, 77.5141_in),
PathPlanner::Point(72.1751_in, 89.774_in),
PathPlanner::Point(67.6271_in, 108.164_in),
PathPlanner::Point(67.4294_in, 122.006_in)
,false),
nullptr},
};
// PathPlanner made path
