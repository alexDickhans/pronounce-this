#pragma once
#include <vector>
#include "velocityProfile/sinusoidalVelocityProfile.hpp"
using namespace Pronounce;
std::vector<std::pair<PathPlanner::BezierSegment, Pronounce::SinusoidalVelocityProfile*>> Auto6BallPath3 = {{PathPlanner::BezierSegment(
PathPlanner::Point(93_in, 90_in),
PathPlanner::Point(89.2429_in, 98.4181_in),
PathPlanner::Point(83.6441_in, 100.254_in),
PathPlanner::Point(81.2712_in, 114.096_in)
,false),
nullptr},
{PathPlanner::BezierSegment(
PathPlanner::Point(81.6667_in, 115.48_in),
PathPlanner::Point(84.435_in, 106.384_in),
PathPlanner::Point(90.7633_in, 95.3105_in),
PathPlanner::Point(100.451_in, 93.3333_in)
,true),
nullptr},
{PathPlanner::BezierSegment(
PathPlanner::Point(107.373_in, 93.7289_in),
PathPlanner::Point(99.2651_in, 90.5648_in),
PathPlanner::Point(92.9379_in, 85.6213_in),
PathPlanner::Point(92.9378_in, 79.6892_in)
,false),
nullptr},
};
// PathPlanner made path
