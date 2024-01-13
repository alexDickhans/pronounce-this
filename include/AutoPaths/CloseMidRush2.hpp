#pragma once
#include <vector>
#include "velocityProfile/sinusoidalVelocityProfile.hpp"
using namespace Pronounce;
std::vector<std::pair<PathPlanner::BezierSegment, Pronounce::SinusoidalVelocityProfile*>> CloseMidPath2 = {{PathPlanner::BezierSegment(
PathPlanner::Point(107.769_in, 40.339_in),
PathPlanner::Point(101.638_in, 54.5762_in),
PathPlanner::Point(97.8814_in, 61.6949_in),
PathPlanner::Point(79.887_in, 66.2429_in)
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
PathPlanner::Point(120.028_in, 15.4237_in),
PathPlanner::Point(121.412_in, 29.0678_in)
,true),
nullptr},
};
// PathPlanner made path
