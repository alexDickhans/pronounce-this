#pragma once
#include <vector>
#include "velocityProfile/sinusoidalVelocityProfile.hpp"
using namespace Pronounce;
std::vector<std::pair<PathPlanner::BezierSegment, Pronounce::SinusoidalVelocityProfile*>> DriverSkills6 = {{PathPlanner::BezierSegment(
PathPlanner::Point(111.921_in, 46.4689_in),
PathPlanner::Point(111.871_in, 37.4718_in),
PathPlanner::Point(112.391_in, 31.6879_in),
PathPlanner::Point(113.701_in, 25.5086_in)
,true),
nullptr},
{PathPlanner::BezierSegment(
PathPlanner::Point(110.536_in, 23.9266_in),
PathPlanner::Point(114.812_in, 14.3855_in),
PathPlanner::Point(125.725_in, 12.0375_in),
PathPlanner::Point(132.486_in, 25.5085_in)
,true),
nullptr},
{PathPlanner::BezierSegment(
PathPlanner::Point(132.091_in, 25.7062_in),
PathPlanner::Point(136.873_in, 39.375_in),
PathPlanner::Point(137.744_in, 51.5236_in),
PathPlanner::Point(138.814_in, 69.8023_in)
,true),
nullptr},
{PathPlanner::BezierSegment(
PathPlanner::Point(137.825_in, 70_in),
PathPlanner::Point(138.893_in, 85.3125_in),
PathPlanner::Point(136.777_in, 112.459_in),
PathPlanner::Point(125.763_in, 123.983_in)
,true),
nullptr},
{PathPlanner::BezierSegment(
PathPlanner::Point(125.763_in, 123.983_in),
PathPlanner::Point(115.539_in, 132.146_in),
PathPlanner::Point(106.414_in, 132.909_in),
PathPlanner::Point(96.1017_in, 132.684_in)
,true),
nullptr},
};
// PathPlanner made path
