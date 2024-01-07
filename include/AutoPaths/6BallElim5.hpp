#pragma once
#include <vector>
#include "velocityProfile/sinusoidalVelocityProfile.hpp"
using namespace Pronounce;
std::vector<std::pair<PathPlanner::BezierSegment, Pronounce::SinusoidalVelocityProfile*>> Auto6BallElim5 = {{PathPlanner::BezierSegment(
PathPlanner::Point(11.8644_in, 69.0113_in),
PathPlanner::Point(11.6666_in, 52.2034_in),
PathPlanner::Point(18.9831_in, 25.7063_in),
PathPlanner::Point(59.9153_in, 25.3108_in)
,true),
nullptr},
};
// PathPlanner made path
