#pragma once
#include <vector>
#include "velocityProfile/sinusoidalVelocityProfile.hpp"
using namespace Pronounce;
std::vector<std::pair<PathPlanner::BezierSegment, Pronounce::SinusoidalVelocityProfile*>> Auto6BallElim5 = {{PathPlanner::BezierSegment(
PathPlanner::Point(11.8644_in, 69.0113_in),
PathPlanner::Point(9.49146_in, 17.9944_in),
PathPlanner::Point(32.4294_in, 18.9831_in),
PathPlanner::Point(59.1243_in, 18.1921_in)
,true),
nullptr},
};
// PathPlanner made path
