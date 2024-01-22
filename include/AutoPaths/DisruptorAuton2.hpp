#pragma once
#include <vector>
#include "velocityProfile/sinusoidalVelocityProfile.hpp"
using namespace Pronounce;
std::vector<std::pair<PathPlanner::BezierSegment, Pronounce::SinusoidalVelocityProfile*>> Disruptor2 = {{PathPlanner::BezierSegment(
PathPlanner::Point(128.531_in, 61.2994_in),
PathPlanner::Point(128.334_in, 38.5593_in),
PathPlanner::Point(126.357_in, 26.8925_in),
PathPlanner::Point(134.068_in, 19.5762_in)
,false),
nullptr},
};
// PathPlanner made path
