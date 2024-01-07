#pragma once
#include <vector>
#include "velocityProfile/sinusoidalVelocityProfile.hpp"
using namespace Pronounce;
std::vector<std::pair<PathPlanner::BezierSegment, Pronounce::SinusoidalVelocityProfile*>> Auto6BallElim1 = {{PathPlanner::BezierSegment(
PathPlanner::Point(17.7966_in, 36.7797_in),
PathPlanner::Point(34.6045_in, 40.9322_in),
PathPlanner::Point(46.4689_in, 75.7345_in),
PathPlanner::Point(64.2656_in, 76.7232_in)
,false),
nullptr},
};
// PathPlanner made path
