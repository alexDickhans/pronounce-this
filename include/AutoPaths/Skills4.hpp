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
PathPlanner::Point(87.3022_in, 100.551_in),
PathPlanner::Point(98.4745_in, 91.356_in)
,false),
nullptr},
{PathPlanner::BezierSegment(
PathPlanner::Point(94.322_in, 75.339_in),
PathPlanner::Point(89.0819_in, 86.7088_in),
PathPlanner::Point(84.2867_in, 102.379_in),
PathPlanner::Point(83.6441_in, 122.994_in)
,true),
nullptr},
};
// PathPlanner made path
