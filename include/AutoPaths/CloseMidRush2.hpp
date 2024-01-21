#pragma once
#include <vector>
#include "velocityProfile/sinusoidalVelocityProfile.hpp"
using namespace Pronounce;
std::vector<std::pair<PathPlanner::BezierSegment, Pronounce::SinusoidalVelocityProfile*>> CloseMidPath2 = {{PathPlanner::BezierSegment(
PathPlanner::Point(110.538_in, 39.548_in),
PathPlanner::Point(105.791_in, 48.644_in),
PathPlanner::Point(89.3786_in, 62.8813_in),
PathPlanner::Point(76.3277_in, 65.6497_in)
,false),
nullptr},
{PathPlanner::BezierSegment(
PathPlanner::Point(79.2938_in, 65.8475_in),
PathPlanner::Point(93.9266_in, 60.904_in),
PathPlanner::Point(91.3559_in, 47.0621_in),
PathPlanner::Point(99.4633_in, 29.0678_in)
,true),
nullptr},
{PathPlanner::BezierSegment(
PathPlanner::Point(99.4633_in, 28.4746_in),
PathPlanner::Point(107.571_in, 10.4802_in),
PathPlanner::Point(117.655_in, 15.8192_in),
PathPlanner::Point(119.039_in, 29.4633_in)
,true),
nullptr},
};
// PathPlanner made path
