#pragma once
#include <vector>
#include "velocityProfile/sinusoidalVelocityProfile.hpp"
using namespace Pronounce;
std::vector<std::pair<PathPlanner::BezierSegment, Pronounce::SinusoidalVelocityProfile*>> SafeCloseAWP1 = {{PathPlanner::BezierSegment(
PathPlanner::Point(14.435_in, 127.345_in),
PathPlanner::Point(4.15254_in, 115.678_in),
PathPlanner::Point(6.32768_in, 103.22_in),
PathPlanner::Point(10.8757_in, 83.6441_in)
,false),
nullptr},
};
// PathPlanner made path
