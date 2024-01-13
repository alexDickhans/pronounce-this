#pragma once
#include <vector>
#include "velocityProfile/sinusoidalVelocityProfile.hpp"
using namespace Pronounce;
std::vector<std::pair<PathPlanner::BezierSegment, Pronounce::SinusoidalVelocityProfile*>> MidRush3 = {{PathPlanner::BezierSegment(
PathPlanner::Point(61.6949_in, 26.4972_in),
PathPlanner::Point(59.5198_in, 54.1808_in),
PathPlanner::Point(54.3785_in, 64.2655_in),
PathPlanner::Point(26.6949_in, 66.0452_in)
,false),
nullptr},
};
// PathPlanner made path
