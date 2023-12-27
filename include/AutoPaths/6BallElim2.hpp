#pragma once
#include <vector>
#include "velocityProfile/sinusoidalVelocityProfile.hpp"
using namespace Pronounce;
std::vector<std::pair<PathPlanner::BezierSegment, Pronounce::SinusoidalVelocityProfile*>> Auto6BallElim2 = {{PathPlanner::BezierSegment(
PathPlanner::Point(69.209_in, 65.452_in),
PathPlanner::Point(69.0113_in, 46.2712_in),
PathPlanner::Point(69.0113_in, 36.7797_in),
PathPlanner::Point(69.209_in, 18.9831_in)
,false),
nullptr},
{PathPlanner::BezierSegment(
PathPlanner::Point(69.209_in, 25.904_in),
PathPlanner::Point(69.4068_in, 34.6045_in),
PathPlanner::Point(62.0904_in, 33.4181_in),
PathPlanner::Point(58.1356_in, 44.4915_in)
,true),
nullptr},
};
// PathPlanner made path
