#pragma once
#include <vector>
#include "velocityProfile/sinusoidalVelocityProfile.hpp"
using namespace Pronounce;
std::vector<std::pair<PathPlanner::BezierSegment, Pronounce::SinusoidalVelocityProfile*>> Disruptor2 = {{PathPlanner::BezierSegment(
PathPlanner::Point(128.531_in, 61.2994_in),
PathPlanner::Point(132.684_in, 26.2994_in),
PathPlanner::Point(119.435_in, 22.3446_in),
PathPlanner::Point(113.107_in, 30.452_in)
,false),
nullptr},
};
// PathPlanner made path
