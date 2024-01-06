#pragma once
#include <vector>
#include "velocityProfile/sinusoidalVelocityProfile.hpp"
using namespace Pronounce;
std::vector<std::pair<PathPlanner::BezierSegment, Pronounce::SinusoidalVelocityProfile*>> CloseAWP2 = {{PathPlanner::BezierSegment(
PathPlanner::Point(100.65_in, 65.452_in),
PathPlanner::Point(99.661_in, 46.8644_in),
PathPlanner::Point(100.254_in, 38.3616_in),
PathPlanner::Point(105.198_in, 25.904_in)
,true),
nullptr},
{PathPlanner::BezierSegment(
PathPlanner::Point(105.198_in, 25.904_in),
PathPlanner::Point(111.921_in, 4.15254_in),
PathPlanner::Point(131.398_in, 10.2825_in),
PathPlanner::Point(132.288_in, 33.6158_in)
,true),
nullptr},
{PathPlanner::BezierSegment(
PathPlanner::Point(132.288_in, 33.6158_in),
PathPlanner::Point(133.178_in, 56.9492_in),
PathPlanner::Point(121.363_in, 55.7627_in),
PathPlanner::Point(121.808_in, 66.0452_in)
,true),
nullptr},
};
// PathPlanner made path
