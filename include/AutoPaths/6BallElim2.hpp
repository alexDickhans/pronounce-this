#pragma once
#include <vector>
#include "velocityProfile/sinusoidalVelocityProfile.hpp"
using namespace Pronounce;
std::vector<std::pair<PathPlanner::BezierSegment, Pronounce::SinusoidalVelocityProfile*>> Auto6BallElim5 = {{PathPlanner::BezierSegment(
PathPlanner::Point(69.4068_in, 25.5085_in),
PathPlanner::Point(66.6384_in, 35.5932_in),
PathPlanner::Point(62.6837_in, 42.5142_in),
PathPlanner::Point(53.7854_in, 43.1074_in)
,true),
nullptr},
};
// PathPlanner made path
