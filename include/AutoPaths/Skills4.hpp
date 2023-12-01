#pragma once
#include <vector>
#include "velocityProfile/sinusoidalVelocityProfile.hpp"
using namespace Pronounce;
std::vector<std::pair<PathPlanner::BezierSegment, Pronounce::SinusoidalVelocityProfile*>> Skills3 = {{PathPlanner::BezierSegment(
PathPlanner::Point(74.9435_in, 77.5141_in),
PathPlanner::Point(72.1751_in, 89.774_in),
PathPlanner::Point(67.6271_in, 108.164_in),
PathPlanner::Point(67.4294_in, 122.006_in)
,true),
nullptr},
{PathPlanner::BezierSegment(
PathPlanner::Point(67.4294_in, 122.006_in),
PathPlanner::Point(68.0227_in, 103.419_in),
PathPlanner::Point(83.1497_in, 84.5342_in),
PathPlanner::Point(94.322_in, 75.339_in)
,false),
nullptr},
{PathPlanner::BezierSegment(
PathPlanner::Point(94.322_in, 75.339_in),
PathPlanner::Point(87.5_in, 86.3133_in),
PathPlanner::Point(79.3432_in, 101.984_in),
PathPlanner::Point(78.7006_in, 122.599_in)
,true),
nullptr},
};
// PathPlanner made path
