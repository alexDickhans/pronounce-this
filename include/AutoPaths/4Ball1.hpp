#pragma once
#include <vector>
#include "velocityProfile/sinusoidalVelocityProfile.hpp"
using namespace Pronounce;
std::vector<std::pair<PathPlanner::BezierSegment, Pronounce::SinusoidalVelocityProfile*>> Auto4Ball1 = {{PathPlanner::BezierSegment(
PathPlanner::Point(59.322_in, 28.8701_in),
PathPlanner::Point(57.7401_in, 49.0395_in),
PathPlanner::Point(40.7345_in, 68.2203_in),
PathPlanner::Point(28.4746_in, 63.6723_in)
,false),
nullptr},
};
// PathPlanner made path
