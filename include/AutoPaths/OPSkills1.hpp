#pragma once
#include <vector>
#include "velocityProfile/sinusoidalVelocityProfile.hpp"
using namespace Pronounce;
std::vector<std::pair<PathPlanner::BezierSegment, Pronounce::SinusoidalVelocityProfile*>> OPSkills1 = {{PathPlanner::BezierSegment(
PathPlanner::Point(27.8814_in, 10.4802_in),
PathPlanner::Point(35_in, 29.8587_in),
PathPlanner::Point(45.0847_in, 38.9548_in),
PathPlanner::Point(45.2825_in, 66.243_in)
,false),
nullptr},
{PathPlanner::BezierSegment(
PathPlanner::Point(45.2825_in, 65.452_in),
PathPlanner::Point(52.4011_in, 48.0508_in),
PathPlanner::Point(63.6724_in, 52.6978_in),
PathPlanner::Point(70_in, 34.4068_in)
,true),
nullptr},
{PathPlanner::BezierSegment(
PathPlanner::Point(69.6045_in, 34.4068_in),
PathPlanner::Point(69.6045_in, 49.1384_in),
PathPlanner::Point(69.3079_in, 60.0636_in),
PathPlanner::Point(69.4068_in, 73.5594_in)
,false),
nullptr},
{PathPlanner::BezierSegment(
PathPlanner::Point(69.4068_in, 66.8362_in),
PathPlanner::Point(73.4604_in, 54.428_in),
PathPlanner::Point(89.1313_in, 60.1378_in),
PathPlanner::Point(94.3225_in, 44.6893_in)
,true),
nullptr},
{PathPlanner::BezierSegment(
PathPlanner::Point(94.3225_in, 44.6893_in),
PathPlanner::Point(94.1748_in, 57.3199_in),
PathPlanner::Point(94.1495_in, 71.2729_in),
PathPlanner::Point(93.7288_in, 97.0904_in)
,false),
nullptr},
};
// PathPlanner made path
