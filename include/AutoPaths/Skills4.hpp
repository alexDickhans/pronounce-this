#pragma once
#include <vector>
#include "velocityProfile/sinusoidalVelocityProfile.hpp"
using namespace Pronounce;
std::vector<std::pair<PathPlanner::BezierSegment, Pronounce::SinusoidalVelocityProfile*>> Skills4 = {{PathPlanner::BezierSegment(
PathPlanner::Point(74.9435_in, 77.5141_in),
PathPlanner::Point(72.1751_in, 89.774_in),
PathPlanner::Point(67.6271_in, 108.164_in),
PathPlanner::Point(67.4294_in, 122.006_in)
,true),
nullptr},
{PathPlanner::BezierSegment(
PathPlanner::Point(67.4294_in, 122.006_in),
PathPlanner::Point(72.5707_in, 102.43_in),
PathPlanner::Point(91.6525_in, 96.003_in),
PathPlanner::Point(106.582_in, 91.7515_in)
,false),
nullptr},
{PathPlanner::BezierSegment(
PathPlanner::Point(102.627_in, 78.3051_in),
PathPlanner::Point(94.4209_in, 88.4885_in),
PathPlanner::Point(84.2867_in, 102.379_in),
PathPlanner::Point(83.6441_in, 122.994_in)
,true),
nullptr},
};
// PathPlanner made path
